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
float out;
float speedL;
float speedR;
float derivative;
float pv;
navigator::navigator(int trig_1, int echo_1, int trig_2, int echo_2, int trig_3, int echo_3, int baz_pin) {
  _trig_1 = trig_1;
  _trig_2 = trig_2;
  _trig_3 = trig_3;
  _echo_1 = echo_1;
  _echo_2 = echo_2;
  _echo_3 = echo_3;
  _baz_pin = baz_pin;
  pinMode(_trig_1, OUTPUT);
  pinMode(_echo_1, INPUT);
  pinMode(_trig_2, OUTPUT);
  pinMode(_echo_2, INPUT);
  pinMode(_trig_3, OUTPUT);
  pinMode(_echo_3, INPUT);
  pinMode(_baz_pin, OUTPUT);
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
  while (!Serial)
    ;

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
      ;
  }

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  calibrateGyro();  // Perform calibration
  lastTime = millis();
  previous_time = micros();
}


float navigator::gyro_value() {
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

void navigator::reset_gyro() {
  angleZ = 0;  // Reset the angle to 0
}


float navigator::ultra_check(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * .0343) / 2;
  return distance;
}


void navigator::run() {
  int us_front = ultra_check(_trig_1, _echo_1);
  int us_left = ultra_check(_trig_2, _echo_2);
  int us_right = ultra_check(_trig_3, _echo_3);
}

void navigator::gyroturn(int sp, int times) {
  float kd = 2;
  float kp = 4;
  float ki = 0;
  float cum_error = 0;
  float last_error = 0;
  for (int i = 0; i < times; i++) {
    int pv = gyro_value();
    int error = sp - pv;
    current_time = micros();
    elapsed_time = current_time - previous_time;
    cum_error += error * elapsed_time;
    if (elapsed_time == 0) {
      rate_error = 0;  //Avoid division by zero
    } else {
      rate_error = (error - last_error) / elapsed_time;
    }
    out = kp * error + ki * cum_error + kd * rate_error;
    if (out > 100) {
      out = 100;
    }
    if (out < 100) {
      out = 100;
    }
    speedL = out;
    speedR = -out;
    //motor stuff
    /*analogWrite(pin,speedL);
    analogWrite(pin,speedR);
    */
    previous_time = current_time;
    last_error = error;
    reset_gyro();
  }
}

void navigator::steer(int ang) {
  float kp = 4;
  float ki = 0.1;
  float kd = 2;
  //def speed = 200;
  float cum_error = 0;
  float last_error = 0;
  while (us_front > 20) {
    pv = gyro_value();
    float error = ang - pv;
    current_time = micros();
    elapsed_time = current_time - previous_time;
    cum_error += error * elapsed_time;
    if (elapsed_time == 0) {  // avoid dividing by zero
      derivative = 0;
    } else {
      rate_error = (error - last_error) / elapsed_time;
    }
    out = kp * error + ki * cum_error + kd * rate_error;
    if (out > 20) {
      out = 20;
    }
    if (out < 20) {
      out = 20;
    }
    /*
    motor stuff
    speedL = defspeed + out;
    speedR = defspeed - out;    
    analogWrite(pin,speedL);
    analogWrite(pin,speedR);
    */
    previous_time = current_time;
    last_error = error;
  }
  //analogWrite(pin,0);
  //analogWrite(pin,0);
  Serial.print('angle =', pv);
}

int navigator::check_us(){
  if (us_left < 20 and us_front < 20 and us_right < 20) {
    return "turn_back";
  } else if (us_front > us_right and us_front > us_left) {
    return "front";
  } else if (us_right > us_left and us_right > us_front){
    return "right";    
  }  else if (us_left > us_right and us_left > us_front){
    return"left";     
  }
}
