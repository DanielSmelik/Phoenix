"""
This code was written by Yuval Rachman, with help from Daniel Smelik. 
Work on this file began on Jan 12, 2025. 

"""
int kp = 1.5;
'''import machine 
from PhoenixRobot import PhoenixRobot as Robot 
import time
from mpu6050 import MPU6050  # Ensure you have the mpu6050.py file in your ESP32 filesystem

class Navigator(Robot):
  # this class navigates in a maze, using ultrasonic sensor.
  def __init__(self):
    super().__init__()'''

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

'''void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100); // small delay to let things settle
}

void loop() {
  degree = gyro();
  Serial.print(degree);
  Serial.print("\n");
}

int gyro(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the X-axis Rotation (Gyroscope X) */
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);  // Gyroscope data for X-axis
  Serial.println(" rad/s");

  delay(500); // Delay for half a second before next reading
  int g.gyro.x = gyro_sensor;
  return gyro_sensor;
}'''
// code for ulrasonic

void setup() {
  pinMode(2, OUTPUT); 
  pinMode(15, INPUT);
  pinMode(4, OUTPUT);
  pinMode(0, INPUT);
  pinMode(33, OUTPUT);
  pinMode(32, INPUT);
  speedL = motor_to_left;
  speedR = motor_to_right;
  Serial.begin(115200);
}
int ultra_check(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int duration = pulseIn(echoPin, HIGH);
  int distance = (duration*.0343)/2;
  return distance;
}

void loop() {
 int us_front = ultra_check(2, 15);
 int us_left = ultra_check(4, 0);
 int us_right = ultra_check(33, 32);
}

def gyroturn(SP, times){
    const int PV;
    const int error;
    for (times):
        PV = gyro_sensor_in5.angle
        eroor = SP - PV
        if eroor > 100:
            eroor = 100
        if eroor < -100:
            eroor = -100
        speedL = eroor
        speedR = -eroor
        tank_drive.on(speedL,speedR)
    print('gyroturn_finished')
    print('angle =',PV)
    gyro_sensor_in5.reset()
    print('gyro has been reseted')
}
