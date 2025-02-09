/*
This code was written by Yuval Rachman, with help from Daniel Smelik. 
Work on this file began on Jan 12, 2025. 


import machine 
from PhoenixRobot import PhoenixRobot as Robot 
import time
from mpu6050 import MPU6050  # Ensure you have the mpu6050.py file in your ESP32 filesystem*/

int kp = 1.5;
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

float angleZ = 0;
unsigned long lastTime = 0;
// Gyroscope calibration offsets
float gyroZoffset = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  calibrateGyro(); // Perform calibration
  lastTime = millis();
  pinMode(2, OUTPUT); 
  pinMode(15, INPUT);
  pinMode(4, OUTPUT);
  pinMode(0, INPUT);
  pinMode(33, OUTPUT);
  pinMode(32, INPUT);
}
void calibrateGyro() {
  int numSamples = 500;
  float sumZ = 0;
  Serial.println("Calibrating gyro... DO NOT MOVE SENSOR");
  delay(1000);
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z;
    delay(10);
  }
  gyroZoffset = sumZ / numSamples;
  Serial.println("Calibration complete!");
}

// code for gyro value
float gyro_value(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  // Apply calibration offsets
  float gyroZ = g.gyro.z - gyroZoffset;
  // Apply a deadband to reduce drift when the sensor is stationary
  if (fabs(gyroZ) < 0.02) {  // threshold; adjust based on your observations
  gyroZ = 0;
  }
  angleZ += gyroZ * dt * (180.0 / PI);
  return angleZ;
}

// code for ulrasonic
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
//just to name it batter for the algorithem
 int us_front = ultra_check(2, 15);
 int us_left = ultra_check(4, 0);
 int us_right = ultra_check(33, 32);

void loop() {

}
