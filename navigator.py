"""
This code was written by Yuval Rachman, with help from Daniel Smelik. 
Work on this file began on Jan 12, 2025. 

"""

'''import machine 
from PhoenixRobot import PhoenixRobot as Robot 
import time
from mpu6050 import MPU6050  # Ensure you have the mpu6050.py file in your ESP32 filesystem

class Navigator(Robot):
  # this class navigates in a maze, using ultrasonic sensor.
  def __init__(self):
    super().__init__()'''

""" Gyro sensor code, not in use rn. 
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
"""



// code for ulrasonic
#define trigPin_1 2
#define echoPin_1 15
#define trigPin_2 4
#define echoPin_2 0
#define trigPin_3 33
#define echoPin_3 32

void setup() {
  pinMode(trigPin_1, OUTPUT); 
  pinMode(echoPin_1, INPUT);
  pinMode(trigPin_2, OUTPUT); 
  pinMode(echoPin_2, INPUT);
  pinMode(trigPin_3, OUTPUT); 
  pinMode(echoPin_3, INPUT);
  Serial.begin(115200);

}
void ultra_front(){
  digitalWrite(trigPin_1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_1, LOW);

  int duration_1 = pulseIn(echoPin_1, HIGH);
  int distance_1 = (duration_1*.0343)/2;
}
void ultra_right(){
  digitalWrite(trigPin_2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_2, LOW);

  int duration_2 = pulseIn(echoPin_2, HIGH);
  int distance_2 = (duration_2*.0343)/2;
}
void ultra_left(){
  digitalWrite(trigPin_3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_3, LOW);

  int duration_3 = pulseIn(echoPin_3, HIGH);
  int distance_3 = (duration_3*.0343)/2;
}


void loop() {

}  
      















