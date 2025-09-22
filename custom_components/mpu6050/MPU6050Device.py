
import logging
import time
import threading
import asyncio

from homeassistant.core import HomeAssistant
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


from scipy.spatial.transform import Rotation as R
from quat import XYZVector as V
import numpy as np

from .const import CONF_BUS_ADDRESS, DOMAIN, CONF_BUS, CONF_ADDRESS, OPTION_ROLL_OFFSET, OPTION_PITCH_OFFSET, OPTION_TARGET_INTERVAL

_LOGGER = logging.getLogger(__name__)


class MPU6050BaseSensor(SensorEntity):
    should_poll = False
    def __init__(self, entry, label, name):
        self._state = 0.0
        self._attr_unique_id = f"{entry.entry_id}_{name}"
        self._attr_name = f"MPU6050 {entry.data[CONF_BUS]}-0x{entry.data[CONF_ADDRESS]:02X} {label}"
        self._attr_device_info = {
            "identifiers": {(DOMAIN, entry.entry_id)},
        }
        _LOGGER.debug(f"MPU6050 Sensor {name} initialisiert.")

    @property
    def state(self):
        return self._state

    def update_state(self, value):
        self._state = round(value, 2)
        _LOGGER.debug(f"MPU6050 Sensor {self._attr_name} aktualisiert: {self._state}")
        self.hass.add_job(self.async_write_ha_state)
    
class MPU6050TempSensor(MPU6050BaseSensor):
    def __init__(self,entry):
        super().__init__(entry, "Temperature", "temp")
        self._attr_unit_of_measurement = "°C"
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

    def turn_on(self, **kwargs):
        self._is_on = True
        self._device.start()
        self.schedule_update_ha_state()

    def turn_off(self, **kwargs):
        self._is_on = False
        self._device.stop()
        self.schedule_update_ha_state()

class MPU6050Device:
    def __init__(self, entry: ConfigEntry):
        self.entry = entry
        self.bus = entry.data[CONF_BUS]
        self.address = entry.data[CONF_ADDRESS]
        self.freq_divider = 0xC7 # 1kHz/(199+1) = 5Hz
        self.freq_s = (self.freq_divider + 1) / 1000.0 # sample frequency in seconds
        self.accel_range = 2.0
        self.sensors = [
            MPU6050BaseSensor(entry, "Acceleration X", "x_accel"),
            MPU6050BaseSensor(entry, "Acceleration Y", "y_accel"),
            MPU6050BaseSensor(entry, "Acceleration Z", "z_accel"),
            MPU6050BaseSensor(entry, "Max Acceleration X", "x_accel_max"),
            MPU6050BaseSensor(entry, "Max Acceleration Y", "y_accel_max"),
            MPU6050BaseSensor(entry, "Max Acceleration Z", "z_accel_max"),
            MPU6050BaseSensor(entry, "Min Acceleration X", "x_accel_min"),
            MPU6050BaseSensor(entry, "Min Acceleration Y", "y_accel_min"),
            MPU6050BaseSensor(entry, "Min Acceleration Z", "z_accel_min"),
            MPU6050BaseSensor(entry, "Pitch", "pitch"),
            MPU6050BaseSensor(entry, "Roll", "roll"),

            MPU6050TempSensor(entry),
        ]
        self.switch = CustomSwitch(self)

        self._enabled = True
        self._pitch = 0.0
        self._roll = 0.0
        
        self._callbacks = set()
        self._loop = asyncio.get_event_loop()
        
        self._stop_event = threading.Event()
        self._thread = None
        self.start()


    @property
    def enabled(self):
        return self._enabled
    
    @property
    def pitch(self):
        return self._pitch
    
    @property
    def roll(self):
        return self._roll

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self.read_sensor_data)
            self._thread.start()
            _LOGGER.info("MPU6050SensorManager gestartet.")
        else:
            _LOGGER.warning(f"Datenlese-Thread läuft bereits. {self._thread.is_alive()}")

    def stop(self):
        if self._thread is not None and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join()
            self._thread = None
            _LOGGER.info("MPU6050SensorManager gestoppt.")

    def read_sensor_data(self):

        while not self._stop_event.is_set():
            _LOGGER.debug("Initialisiere MPU6050...")
    
            p = self.pitch_offset * np.pi / 180.0
            r = self.roll_offset  * np.pi / 180.0

            self.target_interval = self.entry.options.get(OPTION_TARGET_INTERVAL, 1.0) # target interval in seconds
            self.roll_offset = self.entry.options.get(OPTION_ROLL_OFFSET, 0.0)
            self.pitch_offset = self.entry.options.get(OPTION_PITCH_OFFSET, 0.0)
            mpu=None
            try:
                # bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0)

                
                # Make an MPU6050
                mpu = MPU6050(self.bus, self.address, self.freq_divider,a_xGOff=57,a_yGOff=24,a_zGOff=149)
                time.sleep(1) # wait for sensor to stabilize

                # # Initiate your DMP
                # mpu.dmp_initialize()
                # mpu.set_DMP_enabled(True)

            except Exception as e:
                _LOGGER.error(f"Fehler bei der Initialisierung des MPU6050: {e}")
                continue
            
            # packet_size = mpu.DMP_get_FIFO_packet_size()
            # FIFO_buffer = [0]*64

            g = 9.82 # gravity acceleration (m/s^2)
            # r = R.from_euler('y', -90, degrees=True)
            # angle_x, angle_y = 0.0, 0.0

            while not self._stop_event.is_set():
                start_time = time.time()
                target_ct = (int)(self.target_interval / self.freq_s)
                try:
                    ax_sum = 0.0
                    ay_sum = 0.0
                    az_sum = 0.0
                    ax_max, ay_max, az_max, ax_min, ay_min, az_min = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                    for i in range(0,target_ct):
                        time.sleep(max(0, i*self.freq_s - (time.time() - start_time)))
                        accel = mpu.get_acceleration()
                        Ax = accel.x * self.accel_range / 2**15
                        Ay = accel.y * self.accel_range / 2**15
                        Az = accel.z * self.accel_range / 2**15
                        
                        # Apply Ry(-p)
                        x1 = Ax * np.cos(p) - Az * np.sin(p)
                        y1 = Ay
                        z1 = Ax * np.sin(p) + Az * np.cos(p)

                        # Then Rx(-r)
                        x_corr = x1
                        y_corr = y1 * np.cos(r) + z1 * np.sin(r)
                        z_corr = -y1 * np.sin(r) + z1 * np.cos(r)

                        ax_sum += x_corr
                        ay_sum += y_corr
                        az_sum += z_corr
                        ax_max = max(ax_max, x_corr)
                        ay_max = max(ay_max, y_corr)
                        az_max = max(az_max, z_corr)
                        ax_min = min(ax_min, x_corr)
                        ay_min = min(ay_min, y_corr)
                        az_min = min(az_min, z_corr)

                    Ax = ax_sum / target_ct
                    Ay = ay_sum / target_ct
                    Az = az_sum / target_ct


                    roll = -(np.arctan2(Ay, Az) * 180.0 / np.pi)
                    pitch = np.arctan2(-Ax, np.sqrt(Ay**2 + Az**2)) * 180.0 / np.pi


                    self.sensors[0].update_state(Ax)
                    self.sensors[1].update_state(Ay)
                    self.sensors[2].update_state(Az)
                    self.sensors[3].update_state(ax_max)
                    self.sensors[4].update_state(ay_max)
                    self.sensors[5].update_state(az_max)
                    self.sensors[6].update_state(ax_min)
                    self.sensors[7].update_state(ay_min)
                    self.sensors[8].update_state(az_min)
                    self.sensors[9].update_state(pitch)
                    self.sensors[10].update_state(roll)
                    self.sensors[11].update_state(mpu.get_temp())

                    # for i in range(0,20): # infinite loop
                    #     if mpu.isreadyFIFO(packet_size): # Check if FIFO data are ready to use...
                            
                    #         FIFO_buffer = mpu.get_FIFO_bytes(packet_size) # get all the DMP data here
                            
                    #         # # raw acceleration
                    #         # accel = mpu.get_acceleration()
                    #         # Ax = accel.x * 2*g / 2**15
                    #         # Ay = accel.y * 2*g / 2**15
                    #         # Az = accel.z * 2*g / 2**15

                    #         # DMP acceleration (less noisy acceleration - based on fusion)
                    #         accel_dmp = mpu.DMP_get_acceleration_int16(FIFO_buffer)
                    #         Ax_dmp = accel_dmp.x * accel_range*g / 2**15
                    #         Ay_dmp = accel_dmp.y * accel_range*g / 2**15
                    #         Az_dmp = accel_dmp.z * accel_range*g / 2**15

                    #         # # raw gyro (full range: [-250, +250]) (unit: degree / second)
                    #         # gyro = mpu.get_rotation()
                    #         # Gx = gyro.x * 250 / 2**15
                    #         # Gy = gyro.y * 250 / 2**15
                    #         # Gz = gyro.z * 250 / 2**15
                            
                    #         # a_quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
                    #         # quat = R.from_quat((a_quat.x,a_quat.y,a_quat.z,a_quat.w))
                    #         # quat = r * quat
                    #         # rpy = quat.as_euler('zyx', True)
                            
                    #         # yaw   = rpy[0]
                    #         # pitch = rpy[1]
                    #         # roll  = rpy[2]
                    #         # # grav = mpu.DMP_get_gravity(q)
                    #         # roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(q)
                    #         self.sensors[0].update_state(Ax_dmp)
                    #         self.sensors[1].update_state(Ay_dmp)
                    #         self.sensors[2].update_state(Az_dmp)
                    #         self.sensors[3].update_state(mpu.get_temp())
                    #         # print(f'Ax:  {Ax:7.2f}, Ay:  {Ay:7.2f}, Az:  {Az:7.2f}')
                    #         _LOGGER.debug(f'AxD: {Ax_dmp:7.2f}, AyD: {Ay_dmp:7.2f}, AzD: {Az_dmp:7.2f}')
                    #         # _LOGGER.debug(f"Orientation → Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
                    #         _LOGGER.debug(f'temp: {mpu.get_temp()}' )
                            
                    #         break
                    # accel_x = 0
                    # accel_y = 0
                    # accel_z = 0
                    # # repeat readings 10 times and average them

                    # for _ in range(10):
                    #     accel_x += read_raw_data(MPU6050_ACCEL_XOUT_H)
                    #     accel_y += read_raw_data(MPU6050_ACCEL_YOUT_H)
                    #     accel_z += read_raw_data(MPU6050_ACCEL_ZOUT_H)
                    # accel_x /= 10
                    # accel_y /= 10
                    # accel_z /= 10

                    # # normalize Vector
                    # norm = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
                    # if norm != 0:
                    #     accel_x /= norm
                    #     accel_y /= norm
                    #     accel_z /= norm
                    
                    # self.sensors[0].update_state(accel_x)
                    # self.sensors[1].update_state(accel_y)
                    # self.sensors[2].update_state(accel_z)

                    # #gyro_x = read_raw_data(MPU6050_GYRO_XOUT_H) - self.x_offset
                    # #gyro_y = read_raw_data(MPU6050_GYRO_YOUT_H) - self.y_offset

                    # #accel_angle_x = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * (180 / math.pi)
                    # #accel_angle_y = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * (180 / math.pi)

                    # #dt = time.time() - start_time
                    # angle_x = math.atan(accel_x / 16384.0) * (180 / math.pi)
                    # angle_y = math.atan(accel_y / 16384.0) * (180 / math.pi)


                    # for sensor in self.sensors:
                    #     if sensor._sensor_type == "x_angle":
                    #         sensor.update_state(angle_x)
                    #     elif sensor._sensor_type == "y_angle":
                    #         sensor.update_state(angle_y)

                    time.sleep(max(0, self.target_interval - (time.time() - start_time)))
                except Exception as e:
                    _LOGGER.error(f"Fehler beim Lesen der Sensordaten: {e}",stack_info=True, exc_info=True)
                    break
            time.sleep(5) # wait before re-initializing the sensor
