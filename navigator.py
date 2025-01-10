import machine 
from PhoenixRobot import PhoenixRobot as Robot 

class Navigator(Robot):
  def __init__(self):
    super().__init__()
      
from machine import Pin, I2C
import time
from mpu6050 import MPU6050  # Ensure you have the mpu6050.py file in your ESP32 filesystem

# Initialize I2C (use appropriate pins for ESP32)
i2c = I2C(scl=Pin(22), sda=Pin(21))  # ESP32 SCL on GPIO22, SDA on GPIO21

# Create an MPU6050 instance
mpu = MPU6050(i2c)

# Test connection
if mpu.test_connection():
    print("MPU6050 connection successful!")
else:
    print("MPU6050 connection failed.")

# Main loop to read accelerometer and gyroscope data
while True:
    # Read accelerometer data (X, Y, Z)
    accel_data = mpu.get_accel_data()
    print("Accelerometer: X = {:.2f}, Y = {:.2f}, Z = {:.2f}".format(accel_data['x'], accel_data['y'], accel_data['z']))

    # Read gyroscope data (X, Y, Z)
    gyro_data = mpu.get_gyro_data()
    print("Gyroscope: X = {:.2f}, Y = {:.2f}, Z = {:.2f}".format(gyro_data['x'], gyro_data['y'], gyro_data['z']))

    # Sleep for a bit before taking the next reading
    time.sleep(0.5)
