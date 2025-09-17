import smbus2
import math
import time
import json
import argparse

from MPU6050 import MPU6050

from scipy.spatial.transform import Rotation as R
from quat import XYZVector as V
import numpy as np


import threading, queue


def collect(que):
    while True:
        msg = input()
        que.put(msg)

que = queue.Queue()
thread = threading.Thread(target=collect, args=[que], daemon=True).start()

# while thread.is_alive():
#     time.sleep(1)
#     print("The main thread continues while we wait for you...")

# msg = que.get()
# print('You typed:', msg)

# MPU6050 Registers
MPU6050_ADDR = 0x69
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_TEMP_OUT_H = 0x41
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_ACCEL_YOUT_H = 0x3D
MPU6050_ACCEL_ZOUT_H = 0x3F
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_GYRO_YOUT_H = 0x45
MPU6050_GYRO_ZOUT_H = 0x47

# # Configuration
# bus = smbus2.SMBus(1)  # or 0 for RPi 1
# bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0)

# def read_raw_data(addr):
#     # Read raw data in a single transaction
#     high = bus.read_i2c_block_data(MPU6050_ADDR, addr, 2)
#     value = (high[0] << 8) | high[1]
#     if value > 32768:
#         value -= 65536
#     return value

# def parse_args():
#     parser = argparse.ArgumentParser(description="MPU6050 Data Logger")
#     parser.add_argument("--time", type=float, default=1, help="Sleep time in seconds (default: 1)")
#     return parser.parse_args()

# args = parse_args()

# while True:
#     start_time = time.time()
    
#     # Read sensor data
#     accel_x, accel_y, accel_z = [read_raw_data(addr) for addr in (MPU6050_ACCEL_XOUT_H, MPU6050_ACCEL_YOUT_H, MPU6050_ACCEL_ZOUT_H)]
#     gyro_x, gyro_y, gyro_z = [read_raw_data(addr) for addr in (MPU6050_GYRO_XOUT_H, MPU6050_GYRO_YOUT_H, MPU6050_GYRO_ZOUT_H)]
#     temp = read_raw_data(MPU6050_TEMP_OUT_H)
    
#     # Calculate angles
#     x_angle = math.atan(accel_x / 16384.0) * (180 / math.pi)
#     y_angle = math.atan(accel_y / 16384.0) * (180 / math.pi)
    
#     # Prepare data dictionary
#     data = {
#         "x_angle": x_angle,
#         "y_angle": y_angle,
#         "accel_x_raw": accel_x,
#         "accel_y_raw": accel_y,
#         "accel_z_raw": accel_z,
#         "gyro_x_raw": gyro_x,
#         "gyro_y_raw": gyro_y,
#         "gyro_z_raw": gyro_z,
#         "mpu_temp": (temp / 340.0) + 36.53  # Temperature formula for MPU6050
#     }
    
#     # Print JSON data
#     print(json.dumps(data))
    
#     # Adjust sleep time to optimize logging frequency
#     sleep_time = max(0, args.time - (time.time() - start_time))
#     time.sleep(sleep_time)

i2c_bus = 1
device_address = 0x69
freq_divider = 0xC7 # 1hz
#freq_divider = 0x0F


# Make an MPU6050
mpu = MPU6050(i2c_bus, device_address, freq_divider,a_xGOff=57,a_yGOff=24,a_zGOff=149)

# Initiate your DMP
mpu.dmp_initialize()
mpu.set_DMP_enabled(True)

packet_size = mpu.DMP_get_FIFO_packet_size()
FIFO_buffer = [0]*64

g = 9.82 # gravity acceleration (m/s^2)
# r = R.from_euler('y', -90+11.6, degrees=True)
pitch_correction = 11.55
roll_correction = -12
try:

    while True: # infinite loop
        # try:

            
            r = R.from_euler('yx', [-90+pitch_correction,180+roll_correction], degrees=True)
            try:
                msg = que.get(block=False)
                match msg:
                    case 'q':
                        break
                    case 'j':
                        roll_correction-=0.05
                    case 'k':
                        roll_correction+=0.05
                    case _:
                        print(f'got {msg}')
            except queue.Empty:
                pass

            if mpu.isreadyFIFO(packet_size): # Check if FIFO data are ready to use...
                time.sleep(0.01) # wait a bit to avoid reading too fast
                FIFO_buffer = mpu.get_FIFO_bytes(packet_size) # get all the DMP data here
                
                # # raw acceleration
                # accel = mpu.get_acceleration()
                # Ax = accel.x * 2*g / 2**15
                # Ay = accel.y * 2*g / 2**15
                # Az = accel.z * 2*g / 2**15

                # DMP acceleration (less noisy acceleration - based on fusion)
                accel_dmp = mpu.DMP_get_acceleration_int16(FIFO_buffer)
                Ax_dmp = accel_dmp.x * 2*g / 2**15 * 2
                Ay_dmp = accel_dmp.y * 2*g / 2**15 * 2
                Az_dmp = accel_dmp.z * 2*g / 2**15 * 2

                # # raw gyro (full range: [-250, +250]) (unit: degree / second)
                # gyro = mpu.get_rotation()
                # Gx = gyro.x * 250 / 2**15
                # Gy = gyro.y * 250 / 2**15
                # Gz = gyro.z * 250 / 2**15
                
                a_quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
                quat = R.from_quat((a_quat.x,a_quat.y,a_quat.z,a_quat.w))
                quat = r * quat
                rpy = quat.as_euler('zyx', True)
                
                yaw   = rpy[0]
                pitch = rpy[1]
                roll  = rpy[2]
                # # grav = mpu.DMP_get_gravity(q)
                # roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(q)
                

                # print(f'Ax:  {Ax:7.2f}, Ay:  {Ay:7.2f}, Az:  {Az:7.2f}')
                print(f'AxD: {Ax_dmp:7.2f}, AyD: {Ay_dmp:7.2f}, AzD: {Az_dmp:7.2f}')
                # print(f'Grx: {grav.x:7.2f}, Gry: {grav.y:7.2f}, Grz: {grav.z:7.2f}')

                # print(f'Gx: {Gx:7.2f}, Gy: {Gy:7.2f}, Gz: {Gz:7.2f}\n')

                print(f"Orientation → Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
                print(f"correction: pitch {pitch_correction}, roll {roll_correction}")
                print(f"temp: {mpu.get_temp()}")
                print('\n')
                
        # except KeyboardInterrupt:
        #     break
        # except: 
        #     pass   
except KeyboardInterrupt:
    # mpu.set_DMP_enabled(False)
    # mpu.set_sleep_enabled(True)
    pass