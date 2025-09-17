import logging
import math
import time
import threading
import json
import os
import asyncio
from concurrent.futures import ThreadPoolExecutor
from collections import deque
from homeassistant.components.sensor import SensorEntity
from smbus2 import SMBus
from homeassistant.helpers.event import async_track_state_change_event
from .MPU6050 import MPU6050
from .MPUConstants import MPUConstants


from scipy.spatial.transform import Rotation as R
from quat import XYZVector as V
import numpy as np

_LOGGER = logging.getLogger(__name__)

# MPU6050-Register
MPU6050_ADDR = 0x69
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_ACCEL_YOUT_H = 0x3D
MPU6050_ACCEL_ZOUT_H = 0x3F
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_GYRO_YOUT_H = 0x45

# bus = SMBus(1)

i2c_bus = 1
device_address = MPUConstants.MPU6050_ADDRESS_AD0_HIGH # 0x69
freq_divider = 0xC7 # 1kHz/(199+1) = 5Hz
freq_s = (freq_divider + 1) / 1000.0 # sample frequency in seconds

accel_range = 2.0

switch_entity_id = "switch.mpu6050_enabled"

class MPU6050AngleSensor(SensorEntity):
    def __init__(self, name, sensor_type):
        self._name = name
        self._sensor_type = sensor_type
        self._state = None
        self._attr_force_update = True
        self._attr_should_poll = True
        _LOGGER.debug(f"MPU6050AngleSensor {name} initialisiert.")

    @property
    def name(self):
        return self._name

    @property
    def state(self):
        return self._state

    @property
    def unique_id(self):
        return f"mpu6050_{self._sensor_type}"

    def update_state(self, value):
        self._state = round(value, 2)
        _LOGGER.debug(f"{self._sensor_type} Zustand auf {self._state} aktualisiert.")
        self.hass.add_job(self.async_write_ha_state)

class MPU6050SensorManager:
    def __init__(self, hass, sensors, target_interval=1):
        self.hass = hass
        self.sensors = sensors
        self._stop_event = threading.Event()
        self._thread = None
        self.target_interval = target_interval

        self.accel_x_window = deque(maxlen=20)
        self.accel_y_window = deque(maxlen=20)


        # Registrieren des Listeners für den Switch-Zustand
        async_track_state_change_event(
            self.hass, switch_entity_id, self.switch_listener
        )

        _LOGGER.info("MPU6050SensorManager initialisiert.")
        switch_state = hass.states.get(switch_entity_id)
        if switch_state and switch_state.state == "on":
            self.start()

    def switch_listener(self, event):
        new_state = event.data.get("new_state")
        if new_state and new_state.state == "on":
            self.start()
        elif new_state and new_state.state == "off":
            self.stop()

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self.read_sensor_data)
            self._thread.start()
            _LOGGER.info("MPU6050SensorManager gestartet.")
        else:
            _LOGGER.warning("Datenlese-Thread läuft bereits.")
            _LOGGER.warning(self._thread.is_alive())

    def stop(self):
        if self._thread is not None and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join()
            self._thread = None
            _LOGGER.info("MPU6050SensorManager gestoppt.")

    def read_sensor_data(self):

        
        while not self._stop_event.is_set():

            mpu=None
            try:
                # bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0)

                
                # Make an MPU6050
                mpu = MPU6050(i2c_bus, device_address, freq_divider,a_xGOff=57,a_yGOff=24,a_zGOff=149)

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
                target_ct = (int)(self.target_interval / freq_s)
                try:
                    # 5hz frequency
                    ax_sum = 0.0
                    ay_sum = 0.0
                    az_sum = 0.0
                    #ax_max, ay_max, az_max, ax_min, ay_min, az_min = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                    for i in range(0,target_ct):
                        time.sleep(max(0, i*freq_s - (time.time() - start_time)))
                        accel = mpu.get_acceleration()
                        Ax = accel.x * accel_range*g / 2**15
                        Ay = accel.y * accel_range*g / 2**15
                        Az = accel.z * accel_range*g / 2**15
                        ax_sum += Ax
                        ay_sum += Ay
                        az_sum += Az

                    self.sensors[0].update_state(ax_sum / target_ct)
                    self.sensors[1].update_state(ay_sum / target_ct)
                    self.sensors[2].update_state(az_sum / target_ct)
                    self.sensors[3].update_state(mpu.get_temp())

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

def read_raw_data(addr):
    try:
        high = bus.read_byte_data(MPU6050_ADDR, addr)
        low = bus.read_byte_data(MPU6050_ADDR, addr + 1)

        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value
    except Exception as e:
        _LOGGER.error(f"Fehler beim Lesen von Rohdaten von {addr}: {e}")
        return 0

async def async_setup_platform(hass, config, async_add_entities, discovery_info=None):
    sensors = [
        MPU6050AngleSensor("MPU6050 X", "x"),
        MPU6050AngleSensor("MPU6050 Y", "y"),
        MPU6050AngleSensor("MPU6050 Z", "z"),
        MPU6050AngleSensor("MPU6050 Temp", "temp")
    ]

    manager = MPU6050SensorManager(hass, sensors, target_interval=1)
    hass.data["mpu6050_sensor_manager"] = manager
    async_add_entities(sensors)

    _LOGGER.info("MPU6050 Sensor-Plattform erfolgreich eingerichtet.")
