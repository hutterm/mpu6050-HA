
from random import random

from homeassistant.config_entries import ConfigEntry
from concurrent.futures import ThreadPoolExecutor
from collections import deque
from homeassistant.components.sensor import SensorEntity
from homeassistant.components.switch import SwitchEntity
from smbus2 import SMBus
from homeassistant.helpers.event import async_track_state_change_event
from .MPU6050 import MPU6050
from .MPUConstants import MPUConstants
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity import Entity
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.components.sensor import SensorStateClass



from scipy.spatial.transform import Rotation as R
from quat import XYZVector as V
import asyncio
import math
import time
import logging
import random

from .const import (
    CONF_BUS_ADDRESS,
    DOMAIN,
    CONF_BUS,
    CONF_ADDRESS,
    OPTION_ROLL_OFFSET,
    OPTION_PITCH_OFFSET,
    OPTION_TARGET_INTERVAL,
    OPTION_USE_DMP,
    DEFAULT_USE_DMP,
    OPTION_I2C_LOCKS_KEY,
    DEFAULT_I2C_LOCKS_KEY,
)
from .i2c_lock import get_i2c_bus_lock

_LOGGER = logging.getLogger(__name__)

class MPU6050BaseSensor(SensorEntity):
    should_poll = False
    def __init__(self, device, label, name):
        self.device = device
        self.hass = device.hass
        self._state = 0.0
        self._attr_unique_id = f"{device.entry.entry_id}_{name}"
        self._attr_name = f"MPU6050 {device.entry.data[CONF_BUS]}-0x{device.entry.data[CONF_ADDRESS]:02X} {label}"
        self._attr_device_info = {
            "identifiers": {(DOMAIN, device.entry.entry_id)},
        }
        self._attr_state_class = SensorStateClass.MEASUREMENT
        _LOGGER.debug("MPU6050 Sensor %s initialisiert.", name)

    @property
    def native_value(self):
        return self._state

    def update_state(self, value):
        self._state = value
        self.schedule_update_ha_state()
        _LOGGER.debug("MPU6050 Sensor %s aktualisiert: %s", self._attr_name, self._state)


class MPU6050TempSensor(MPU6050BaseSensor):
    def __init__(self, device):
        super().__init__(device, "Temperature", "temp")
        self._attr_native_unit_of_measurement = "°C"
        self._attr_icon = "mdi:thermometer"
        self._attr_device_class = "temperature"

class CustomSwitch(SwitchEntity):
    def __init__(self, device):
        self._is_on = True
        self._device = device
        self._attr_name = f"MPU6050 {device.entry.data[CONF_BUS]}-0x{device.entry.data[CONF_ADDRESS]:02X} Enabled"
        self._attr_unique_id = f"{device.entry.entry_id}_enabled"
        self._attr_device_info = {
            "identifiers": {(DOMAIN, device.entry.entry_id)},
            "name": f"MPU6050 {device.entry.data[CONF_BUS]}:0x{device.entry.data[CONF_ADDRESS]:02X}",
            "manufacturer": "InvenSense",
        }
        self._attr_icon = "mdi:power"

    @property
    def is_on(self):
        return self._is_on

    async def async_turn_on(self, **kwargs):
        self._is_on = True
        self._device.start()
        self.schedule_update_ha_state()

    async def async_turn_off(self, **kwargs):
        self._is_on = False
        self._device.stop()
        self.schedule_update_ha_state()

class MPU6050Device:
    def __init__(self, hass: HomeAssistant, entry: ConfigEntry):
        self.hass = hass
        self.entry = entry
        self.bus = entry.data[CONF_BUS]
        self.address = entry.data[CONF_ADDRESS]
        self.raw_freq_divider = 0x31  # 1kHz/(49+1) = 20Hz for complementary filter
        self.raw_freq_s = (self.raw_freq_divider + 1) / 1000.0
        self.freq_divider = self.raw_freq_divider
        self.freq_s = self.raw_freq_s
        self.dlpf_mode = MPUConstants.MPU6050_DLPF_BW_42
        self.temp_freq = 10.0 # read temperature every 10 seconds
        self.accel_range = 2.0
        self.gyro_lsb_per_dps = 131.0  # FS=250dps in initialize()
        self._complementary_tau_s = 0.75
        self._accel_trust_tolerance_g = 0.08
        self._filter_last_ts: float | None = None
        self._use_dmp = DEFAULT_USE_DMP
        self.sensors = [
            MPU6050BaseSensor(self, "Acceleration X", "x_accel"),
            MPU6050BaseSensor(self, "Acceleration Y", "y_accel"),
            MPU6050BaseSensor(self, "Acceleration Z", "z_accel"),
            MPU6050BaseSensor(self, "Max Acceleration X", "x_accel_max"),
            MPU6050BaseSensor(self, "Max Acceleration Y", "y_accel_max"),
            MPU6050BaseSensor(self, "Max Acceleration Z", "z_accel_max"),
            MPU6050BaseSensor(self, "Min Acceleration X", "x_accel_min"),
            MPU6050BaseSensor(self, "Min Acceleration Y", "y_accel_min"),
            MPU6050BaseSensor(self, "Min Acceleration Z", "z_accel_min"),
            MPU6050BaseSensor(self, "Pitch", "pitch"),
            MPU6050BaseSensor(self, "Roll", "roll"),

            MPU6050TempSensor(self),
        ]
        self.switch = CustomSwitch(self)

        self._pitch = 0.0
        self._roll = 0.0

        self._task: asyncio.Task | None = None
        self._stop_event = asyncio.Event()
        
    
    @property
    def pitch(self):
        return self._pitch
    
    @property
    def roll(self):
        return self._roll

    def _get_bus_lock(self):
        i2c_locks_key = self.entry.options.get(
            OPTION_I2C_LOCKS_KEY,
            DEFAULT_I2C_LOCKS_KEY,
        )
        lock, created = get_i2c_bus_lock(self.hass, i2c_locks_key, self.bus)
        if created:
            _LOGGER.warning("MPU6050 created new lock for I2C bus %s", self.bus)
        return lock

    async def _run_i2c_call(self, lock, func, *args):
        async with lock:
            return await self.hass.async_add_executor_job(func, *args)

    def _get_runtime_options(self):
        try:
            target_interval = float(
                self.entry.options.get(
                    OPTION_TARGET_INTERVAL,
                    1.0,
                )
            )
        except (TypeError, ValueError):
            target_interval = 1.0
        target_interval = max(0.1, min(10.0, target_interval))

        try:
            roll_offset = float(self.entry.options.get(OPTION_ROLL_OFFSET, 0.0))
        except (TypeError, ValueError):
            roll_offset = 0.0

        try:
            pitch_offset = float(self.entry.options.get(OPTION_PITCH_OFFSET, 0.0))
        except (TypeError, ValueError):
            pitch_offset = 0.0

        use_dmp = bool(self.entry.options.get(OPTION_USE_DMP, DEFAULT_USE_DMP))
        return target_interval, roll_offset, pitch_offset, use_dmp

    @staticmethod
    def _dmp_divider_for_interval(interval_s: float) -> int:
        # DMP FIFO rate = 200Hz / (1 + divider)
        divider = int(round((200.0 * interval_s) - 1.0))
        return max(0, min(255, divider))

    @staticmethod
    def _rotation_terms(pitch_offset: float, roll_offset: float):
        pitch_r = pitch_offset * math.pi / 180.0
        roll_r = roll_offset * math.pi / 180.0
        return math.cos(pitch_r), math.sin(pitch_r), math.cos(roll_r), math.sin(roll_r)

    @staticmethod
    def _rotate_vector(x: float, y: float, z: float, cp: float, sp: float, cr: float, sr: float):
        # First Ry(-pitch), then Rx(-roll) to align to vehicle frame.
        x1 = x * cp - z * sp
        y1 = y
        z1 = x * sp + z * cp
        return x1, y1 * cr + z1 * sr, -y1 * sr + z1 * cr

    @staticmethod
    def _accel_to_angles(ax: float, ay: float, az: float):
        roll = -(math.atan2(ay, az) * 180.0 / math.pi)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180.0 / math.pi
        return roll, pitch

    @staticmethod
    def _wrap_angle_deg(angle: float) -> float:
        return ((angle + 180.0) % 360.0) - 180.0

    @classmethod
    def _blend_angle_deg(cls, source: float, target: float, target_weight: float) -> float:
        target_weight = max(0.0, min(1.0, target_weight))
        delta = cls._wrap_angle_deg(target - source)
        return cls._wrap_angle_deg(source + target_weight * delta)

    def _update_complementary_filter(
        self,
        roll_accel: float,
        pitch_accel: float,
        gx_dps: float,
        gy_dps: float,
        dt_s: float,
        accel_norm_g: float,
    ) -> None:
        gyro_weight = self._complementary_tau_s / (self._complementary_tau_s + dt_s)
        accel_weight = 1.0 - gyro_weight

        norm_error = abs(accel_norm_g - 1.0)
        trust = 1.0 - min(1.0, norm_error / (2.0 * self._accel_trust_tolerance_g))
        accel_weight *= trust

        roll_gyro = self._wrap_angle_deg(self._roll + gx_dps * dt_s)
        pitch_gyro = self._wrap_angle_deg(self._pitch + gy_dps * dt_s)

        self._roll = self._blend_angle_deg(roll_gyro, roll_accel, accel_weight)
        self._pitch = self._blend_angle_deg(pitch_gyro, pitch_accel, accel_weight)

    def _publish_sample(
        self,
        ax: float,
        ay: float,
        az: float,
        ax_max: float,
        ay_max: float,
        az_max: float,
        ax_min: float,
        ay_min: float,
        az_min: float,
        pitch: float,
        roll: float,
    ) -> None:
        self._pitch = pitch
        self._roll = roll
        self.sensors[0].update_state(ax)
        self.sensors[1].update_state(ay)
        self.sensors[2].update_state(az)
        self.sensors[3].update_state(ax_max)
        self.sensors[4].update_state(ay_max)
        self.sensors[5].update_state(az_max)
        self.sensors[6].update_state(ax_min)
        self.sensors[7].update_state(ay_min)
        self.sensors[8].update_state(az_min)
        self.sensors[9].update_state(pitch)
        self.sensors[10].update_state(roll)

    async def _read_complementary_window(self, lock, mpu, target_interval, cp, sp, cr, sr):
        target_ct = max(1, int(round(target_interval / self.raw_freq_s)))
        window_start = time.monotonic()

        ax_sum = ay_sum = az_sum = 0.0
        ax_max = ay_max = az_max = float("-inf")
        ax_min = ay_min = az_min = float("inf")

        for i in range(target_ct):
            await asyncio.sleep(
                max(
                    0.0,
                    i * self.raw_freq_s
                    + random.uniform(0, 0.01)
                    - (time.monotonic() - window_start),
                )
            )
            accel = await self._run_i2c_call(lock, mpu.get_acceleration)
            gyro = await self._run_i2c_call(lock, mpu.get_rotation)

            ax = accel.x * self.accel_range / (2**15)
            ay = accel.y * self.accel_range / (2**15)
            az = accel.z * self.accel_range / (2**15)
            gx = gyro.x / self.gyro_lsb_per_dps
            gy = gyro.y / self.gyro_lsb_per_dps
            gz = gyro.z / self.gyro_lsb_per_dps

            ax, ay, az = self._rotate_vector(ax, ay, az, cp, sp, cr, sr)
            gx, gy, _ = self._rotate_vector(gx, gy, gz, cp, sp, cr, sr)

            roll_accel, pitch_accel = self._accel_to_angles(ax, ay, az)
            now = time.monotonic()
            if self._filter_last_ts is None:
                self._roll = roll_accel
                self._pitch = pitch_accel
                dt_s = self.raw_freq_s
            else:
                dt_s = max(0.001, min(0.5, now - self._filter_last_ts))
            self._filter_last_ts = now

            accel_norm_g = math.sqrt(ax * ax + ay * ay + az * az)
            self._update_complementary_filter(
                roll_accel,
                pitch_accel,
                gx,
                gy,
                dt_s,
                accel_norm_g,
            )

            ax_sum += ax
            ay_sum += ay
            az_sum += az
            ax_max = max(ax_max, ax)
            ay_max = max(ay_max, ay)
            az_max = max(az_max, az)
            ax_min = min(ax_min, ax)
            ay_min = min(ay_min, ay)
            az_min = min(az_min, az)

        return (
            ax_sum / target_ct,
            ay_sum / target_ct,
            az_sum / target_ct,
            ax_max,
            ay_max,
            az_max,
            ax_min,
            ay_min,
            az_min,
            self._pitch,
            self._roll,
        )

    async def _read_dmp_sample(
        self,
        lock,
        mpu,
        packet_size: int,
        cp: float,
        sp: float,
        cr: float,
        sr: float,
        roll_offset: float,
        pitch_offset: float,
    ):
        int_status = await self._run_i2c_call(lock, mpu.get_int_status)
        fifo_count = await self._run_i2c_call(lock, mpu.get_FIFO_count)

        if fifo_count == 1024 or (int_status & 0x10):
            await self._run_i2c_call(lock, mpu.reset_FIFO)
            _LOGGER.debug("MPU6050 FIFO overflow, reset performed")
            return None

        if fifo_count < packet_size:
            return None

        ax_sum = ay_sum = az_sum = 0.0
        ax_max = ay_max = az_max = float("-inf")
        ax_min = ay_min = az_min = float("inf")
        packet_count = 0
        latest_roll = self._roll
        latest_pitch = self._pitch

        while fifo_count >= packet_size:
            fifo_buffer = await self._run_i2c_call(lock, mpu.get_FIFO_bytes, packet_size)
            fifo_count -= packet_size

            accel = mpu.DMP_get_acceleration_int16(fifo_buffer)
            quat = mpu.DMP_get_quaternion(fifo_buffer)
            dmp_rpy = mpu.DMP_get_euler_roll_pitch_yaw(quat)

            ax = accel.x * self.accel_range / (2**15)
            ay = accel.y * self.accel_range / (2**15)
            az = accel.z * self.accel_range / (2**15)
            ax, ay, az = self._rotate_vector(ax, ay, az, cp, sp, cr, sr)

            ax_sum += ax
            ay_sum += ay
            az_sum += az
            ax_max = max(ax_max, ax)
            ay_max = max(ay_max, ay)
            az_max = max(az_max, az)
            ax_min = min(ax_min, ax)
            ay_min = min(ay_min, ay)
            az_min = min(az_min, az)
            packet_count += 1

            latest_roll = self._wrap_angle_deg(dmp_rpy.x - roll_offset)
            latest_pitch = self._wrap_angle_deg(dmp_rpy.y - pitch_offset)

        if packet_count == 0:
            return None

        return (
            ax_sum / packet_count,
            ay_sum / packet_count,
            az_sum / packet_count,
            ax_max,
            ay_max,
            az_max,
            ax_min,
            ay_min,
            az_min,
            latest_pitch,
            latest_roll,
        )

    def start(self):
        if self._task is None or self._task.done():
            self._stop_event.clear()
            self._task = self.hass.loop.create_task(self._run_loop())
            _LOGGER.info("MPU6050SensorManager gestartet.")
        else:
            _LOGGER.warning("Datenlese-Thread läuft bereits. %s", self._task is not None and not self._task.done())

    def stop(self):
        if self._task:
            self._stop_event.set()
            self._task.cancel()
            self._task = None
            _LOGGER.info("MPU6050SensorManager gestoppt.")
        else:
            _LOGGER.warning("Datenlese-Thread ist nicht aktiv.")

    async def _run_loop(self):
        backoff = 1
        while not self._stop_event.is_set():
            self.target_interval, self.roll_offset, self.pitch_offset, self._use_dmp = (
                self._get_runtime_options()
            )
            alock = self._get_bus_lock()
            cp, sp, cr, sr = self._rotation_terms(self.pitch_offset, self.roll_offset)
            mpu = None
            packet_size = 0

            if self._use_dmp:
                self.freq_divider = self._dmp_divider_for_interval(self.target_interval)
                self.freq_s = (self.freq_divider + 1) / 200.0
            else:
                self.freq_divider = self.raw_freq_divider
                self.freq_s = self.raw_freq_s

            try:
                mpu = await self.hass.async_add_executor_job(
                    MPU6050,
                    self.bus,
                    self.address,
                    self.freq_divider,
                )
                await asyncio.sleep(2) # wait for sensor to power up
                if self._use_dmp:
                    await self._run_i2c_call(alock, mpu.dmp_initialize)
                    await self._run_i2c_call(alock, mpu.set_DMP_enabled, True)
                    packet_size = await self._run_i2c_call(
                        alock, mpu.DMP_get_FIFO_packet_size
                    )
                    if packet_size <= 0:
                        raise RuntimeError("Invalid DMP packet size")
                    await self._run_i2c_call(alock, mpu.reset_FIFO)
                    _LOGGER.info(
                        "MPU6050 running in DMP mode (%.2f Hz FIFO)",
                        200.0 / (self.freq_divider + 1),
                    )
                else:
                    await self._run_i2c_call(
                        alock,
                        mpu.initialize,
                        None,
                        None,
                        None,
                        57,
                        24,
                        149,
                    )
                    await self._run_i2c_call(alock, mpu.set_DLF_mode, self.dlpf_mode)
                    await self._run_i2c_call(alock, mpu.set_rate, self.freq_divider)
                    seed_accel = await self._run_i2c_call(alock, mpu.get_acceleration)
                    ax = seed_accel.x * self.accel_range / (2**15)
                    ay = seed_accel.y * self.accel_range / (2**15)
                    az = seed_accel.z * self.accel_range / (2**15)
                    ax, ay, az = self._rotate_vector(ax, ay, az, cp, sp, cr, sr)
                    self._roll, self._pitch = self._accel_to_angles(ax, ay, az)
                    self._filter_last_ts = time.monotonic()
                    _LOGGER.info(
                        "MPU6050 running in complementary mode (%.2f Hz internal)",
                        1000.0 / (self.freq_divider + 1),
                    )
                await asyncio.sleep(1) # wait for sensor to stabilize

                backoff = 1
            except Exception as e:
                _LOGGER.error("Init error: %s, retry in %s s", e, backoff)
                if mpu is not None:
                    try:
                        await self.hass.async_add_executor_job(mpu.close)
                    except OSError as close_error:
                        _LOGGER.warning("I2C close error during init retry: %s", close_error)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 60)
                continue

            temp_timer = time.monotonic()

            try:
                while not self._stop_event.is_set():
                    start = time.monotonic()
                    try:
                        (
                            current_target_interval,
                            current_roll_offset,
                            current_pitch_offset,
                            current_use_dmp,
                        ) = self._get_runtime_options()
                        if (
                            current_use_dmp != self._use_dmp
                            or abs(current_target_interval - self.target_interval) > 1e-6
                            or abs(current_roll_offset - self.roll_offset) > 1e-6
                            or abs(current_pitch_offset - self.pitch_offset) > 1e-6
                        ):
                            _LOGGER.info("MPU6050 options changed, reinitializing")
                            break

                        if self._use_dmp:
                            sample = await self._read_dmp_sample(
                                alock,
                                mpu,
                                packet_size,
                                cp,
                                sp,
                                cr,
                                sr,
                                self.roll_offset,
                                self.pitch_offset,
                            )
                        else:
                            sample = await self._read_complementary_window(
                                alock,
                                mpu,
                                self.target_interval,
                                cp,
                                sp,
                                cr,
                                sr,
                            )

                        if sample is not None:
                            self._publish_sample(*sample)

                        if time.monotonic() - temp_timer >= self.temp_freq:
                            self.sensors[11].update_state(
                                await self._run_i2c_call(alock, mpu.get_temp)
                            )
                            temp_timer = time.monotonic()

                        await asyncio.sleep(
                            max(
                                0,
                                self.target_interval
                                + random.uniform(0, 0.01)
                                - (time.monotonic() - start),
                            )
                        )
                    except Exception as e:
                        _LOGGER.exception("Read error: %s", e)
                        break
            finally:
                if mpu is not None:
                    try:
                        await self.hass.async_add_executor_job(mpu.close)
                    except OSError as close_error:
                        _LOGGER.warning("I2C close error: %s", close_error)
                self._filter_last_ts = None

            if not self._stop_event.is_set():
                await asyncio.sleep(5) # wait before re-initializing the sensor
