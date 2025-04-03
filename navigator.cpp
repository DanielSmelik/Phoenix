#include "navigator.h" 
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

 Adafruit_MPU6050 mpu;
  
  float angleZ = 0;
  float gyroZoffset = 0;
  unsigned long lastTime = 0;
  unsigned long previous_time;
  unsigned long current_time;
  unsigned long elapsed_time;
  float rate_error;
navigator::navigator(int test_param){
  //_test_param = test_param;
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

void navigator::begin() {
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
  previous_time = micros();
  

}


float navigator::gyro_value(){
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

float navigator::ultra_check(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration*.0343)/2;
  return distance;
}
  

void navigator::run(){
  int us_front = ultra_check(2, 15);
  int us_left = ultra_check(4, 0);
  int us_right = ultra_check(33, 32);  
}

void navigator::gyroturn(int sp, int times){
  int KD = 2;
  int KP = 4;
  int KI = 0;
  float cum_error = 0;
  float last_error = 0;
  for (int i = 0; i < times; i++){
    int pv = gyro_value();
    int error = sp - pv;
    current_time = micros();
    elapsed_time = current_time - previous_time;
    cum_error += error * elapsed_time;
    if (elapsed_time == 0){
      rate_error = 0;  //Avoid division by zero
    }
    else{
      rate_error = (error - last_error) / elapsed_time;
    }
    float out = KP * error + KI * cum_error + KD * rate_error;
    if (out > 100);
    out = 100;
    if (out < 100);
    out = 100;
  


  }

}





