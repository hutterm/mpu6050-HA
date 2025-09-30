
import logging
from random import random
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
from homeassistant.components.sensor import SensorStateClass
from homeassistant.const import TEMP_CELSIUS



from scipy.spatial.transform import Rotation as R
from quat import XYZVector as V
import numpy as np

from .const import CONF_BUS_ADDRESS, DOMAIN, CONF_BUS, CONF_ADDRESS, OPTION_ROLL_OFFSET, OPTION_PITCH_OFFSET, OPTION_TARGET_INTERVAL

_LOGGER = logging.getLogger(__name__)


class MPU6050BaseSensor(SensorEntity):
    should_poll = False
    def __init__(self, device, label, name):
        self._state = 0.0
        self._attr_unique_id = f"{device.entry.entry_id}_{name}"
        self._attr_name = f"MPU6050 {device.entry.data[CONF_BUS]}-0x{device.entry.data[CONF_ADDRESS]:02X} {label}"
        self._attr_device_info = {
            "identifiers": {(DOMAIN, device.entry.entry_id)},
        }
        self._attr_state_class = SensorStateClass.MEASUREMENT
        _LOGGER.debug(f"MPU6050 Sensor {name} initialisiert.")

    @property
    def state(self):
        return self._state

    def update_state(self, value):
        self._state = value
        _LOGGER.debug(f"MPU6050 Sensor {self._attr_name} aktualisiert: {self._state}")
        self.hass.add_job(self.async_write_ha_state)
    
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

    def turn_on(self, **kwargs):
        self._is_on = True
        self._device.start()
        self.schedule_update_ha_state()

    def turn_off(self, **kwargs):
        self._is_on = False
        self._device.stop()
        self.schedule_update_ha_state()

class MPU6050Device:
    def __init__(self, hass: HomeAssistant, entry: ConfigEntry):
        self.hass = hass
        self.entry = entry
        self.bus = entry.data[CONF_BUS]
        self.address = entry.data[CONF_ADDRESS]
        self.freq_divider = 0xC7 # 1kHz/(199+1) = 5Hz
        self.freq_s = (self.freq_divider + 1) / 1000.0 # sample frequency in seconds
        self.temp_freq = 10.0 # read temperature every 10 seconds
        self.accel_range = 2.0
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
        else:
            _LOGGER.warning("Datenlese-Thread ist nicht aktiv.")

    def read_sensor_data(self):

        while not self._stop_event.is_set():
            _LOGGER.debug("Initialisiere MPU6050...")

            self.target_interval = self.entry.options.get(OPTION_TARGET_INTERVAL, 1.0) # target interval in seconds
            self.roll_offset = self.entry.options.get(OPTION_ROLL_OFFSET, 0.0)
            self.pitch_offset = self.entry.options.get(OPTION_PITCH_OFFSET, 0.0)
            
            p = self.pitch_offset * np.pi / 180.0
            r = self.roll_offset  * np.pi / 180.0

            mpu=None
            try:
                # bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0)

                
                # Make an MPU6050
                mpu = MPU6050(self.bus, self.address, self.freq_divider,a_xGOff=57,a_yGOff=24,a_zGOff=149)
                time.sleep(1) # wait for sensor to stabilize

            except Exception as e:
                _LOGGER.error(f"Fehler bei der Initialisierung des MPU6050: {e}")
                continue
            

            g = 9.82 # gravity acceleration (m/s^2)

            temp_timer = time.time()

            while not self._stop_event.is_set():
                start_time = time.time()

                target_ct = (int)(self.target_interval / self.freq_s)
                try:
                    ax_sum = 0.0
                    ay_sum = 0.0
                    az_sum = 0.0
                    ax_max, ay_max, az_max, ax_min, ay_min, az_min = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                    for i in range(0,target_ct):
                        # add small random offset to the wait time to avoid always reading at the same time
                        # this helps to avoid interference with other I2C devices
                        time.sleep(max(0, i*self.freq_s + random.uniform(0, 0.01) - (time.time() - start_time)))
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
                    if time.time() - temp_timer >= self.temp_freq:
                        temp_timer = time.time()
                        self.sensors[11].update_state(mpu.get_temp())

                    time.sleep(max(0, self.target_interval + random.uniform(0, 0.01) - (time.time() - start_time)))
                except Exception as e:
                    _LOGGER.error(f"Fehler beim Lesen der Sensordaten: {e}",stack_info=True, exc_info=True)
                    break
            time.sleep(5) # wait before re-initializing the sensor
