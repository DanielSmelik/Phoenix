#include "phoenix_robot.h" 
#include "Arduino.h"
#include "wms.h"
#include <Adafruit_NeoPixel.h>
#include <iarduino_HC_SR04_tmr.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define ring_pin 2
#define ring_length 12

const unsigned int length = 20; //space from robot to walls.
const unsigned int MAX_MESSAGE_LENGTH = 64; //max message for cli. 


//for gyro 
float angleZ = 0;
float gyroZoffset = 0;
//for angle calculation 
unsigned long lastTime = 0;
unsigned long previous_time;

//for turning to an angle. 
unsigned long current_time;
unsigned long elapsed_time;
float rate_error;
float out;
float speedL;
float speedR;
float derivative;
float pv;

Adafruit_NeoPixel ring(ring_length, ring_pin, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;

void reset_gyro(){
  angleZ = 0;  // after turning you want to reset the angle to 0; 
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

Phoenix::Phoenix(int _button_pin){
  const int button_pin =_button_pin;
  pinMode(ring_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);
}

//int flamePins[] = {3, 5, 6, 9, 10}; 
//const int numFlameSensors = 5;
//const int flameThreshold = 2000;

//first goes the trig pin, then the echo: 
iarduino_HC_SR04_tmr us1(12,11); // front ultrasonic 
iarduino_HC_SR04_tmr us2(10,9); //right ultrasonic 
iarduino_HC_SR04_tmr us3(8,7); //left ultrasonic 


void Phoenix::begin(){
  Serial.begin(115200);
  

  ring.begin();
  ring.show();
  ring.setBrightness(35);

  for (int i =0; i < ring_length; i++){
    ring.setPixelColor(i, ring.Color(255, 0, 0));
  }
  ring.show();
  
  Serial.print("\n");
  Serial.println("Phoenix Robot started...");
  delay(1000);
  while (WMS_Begin(MOTORSHIELD_AD11, MOTORSHIELD_PWM_RES_9BIT, 3900) != true) {
      Serial.println("Motor Shield is not reachable");
      delay(2000);
  }
  Serial.println("Motors are ON...");
  
   if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  calibrateGyro();

  for (int i =0; i < ring_length; i++){
    ring.setPixelColor(i, ring.Color(0, 255, 0));
  }
  us1.begin(100);
  us2.begin(100);
  us3.begin(100);
  ring.show();
  }

void Phoenix::motgo(int speedl, int speedr){
  DriveAB(speedl, speedr);  
  Serial.print("Motors set at speed: "); Serial.print('\t'); Serial.print(speedl); Serial.print('\t'); Serial.println(speedr);

}

void Phoenix::steer(int ang, float kp, float ki, float kd, int defspeed){
  float cum_error = 0;
  float last_error = 0;
  pv = get_anglez();
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
    else if (out < 20) {
      out = 20;
    }
    speedL = defspeed + out;
    speedR = defspeed - out;    
    motgo(speedL, speedR);
    
    previous_time = current_time;
    last_error = error;
}

void Phoenix::gyroturn(int sp, int times, const float kp, const float ki, const float kd){
  float cum_error = 0;
  float last_error = 0;
  for (int i = 0; i < times; i++) {
    int pv = get_anglez();
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
    if (out > 512) {
      out = 512;
    }
    if (out < -512) {
      out = -512;
    }
    speedL = out;
    speedR = -out;
    motgo(speedL, speedR);    
    previous_time = current_time;
    last_error = error;
    reset_gyro();
  }
}

void Phoenix::motbrake(){
  BrakeAB();
}

float Phoenix::get_anglez() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  // Apply calibration offsets
  float gyroZ = g.gyro.z - gyroZoffset;
  // Apply a deadband to reduce drift when the sensor is stationary
  if (fabs(gyroZ) < 0.02) {  
    gyroZ = 0;
  }
  angleZ += gyroZ * dt * (180.0 / PI);
  return angleZ;
}

void Phoenix::blinkRing(){}

int Phoenix::check_us(){
  Serial.print("Front Dist: ");Serial.print(us1.distance()); Serial.print("\t");
  Serial.print("Right Dist: ");Serial.print(us2.distance()); Serial.print("\t");
  Serial.print("Left Dist: ");Serial.print(us3.distance()); Serial.println();
}


char Phoenix::get_dir(){
  if (us1.distance() > length && us2.distance() <= length && us3.distance() <= length){
    return 'f';
  }
  else if (us1.distance() <= length && us2.distance() > length && us3.distance() <= length){
    return 'r';
  }
  else if (us1.distance() <= length && us2.distance() <= length && us3.distance() > length){
    return 'l';
  }
  else if (us1.distance() <= length && us2.distance() <= length && us3.distance() <= length){
    return 'b';
  }
  else {return 'e';}
}

void Phoenix::readcli() {
     while (Serial.available() > 0) { 
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;

    char inByte = Serial.read();

    // Build message until newline or max length reached
    if (inByte != '\n' && message_pos < MAX_MESSAGE_LENGTH - 1) {
      message[message_pos++] = inByte;
    } else {
      // End of message
      message[message_pos] = '\0'; // Null-terminate
      Serial.println(message);     // Echo the message

      // Reset position for next message
      message_pos = 0;

      // Temporary copy for parsing
      char tmpmsg[MAX_MESSAGE_LENGTH];
      strcpy(tmpmsg, message);

      char *ptr = strtok(tmpmsg, " ");
      char cmd = '\0';
      int command[4] = {0};
      int argindex = 0;

      // Tokenize and parse arguments
      while (ptr != NULL && argindex < 4) {
        if (argindex == 0) {
          cmd = ptr[0]; // First token is command character
        } else {
          command[argindex] = atoi(ptr); // Convert argument to int
        }
        argindex++;
        ptr = strtok(NULL, " ");
      }

      // Execute command
      switch (cmd) {
        /*case 'h': // Set pin HIGH
          pinMode(command[1], OUTPUT);
          digitalWrite(command[1], HIGH);
          Serial.print("Pin "); Serial.print(command[1]); Serial.println(" is SET");
          break;

        case 'l': // Set pin LOW
          pinMode(command[1], OUTPUT);
          digitalWrite(command[1], LOW);
          Serial.print("Pin "); Serial.print(command[1]); Serial.println(" is RESET");
          break;
        */
        case 'm': //  Both motors control at given speed.
          motgo(command[1], command[1]);
          break;

        case 'R': //  turn right at speed.
          motgo(command[1], -command[1]);
          break;
        case 'L':
          DriveAB(-command[1], command[1]);
          break;
        
        case 'b':
          BrakeAB();
          break;
        case 'g':
          Serial.print("Current angle (z): "); Serial.println(get_anglez());
          break;
        case 'u':
          Serial.print("Direction is: "); Serial.println(get_dir());

        /*case 'd': // Digital read
          pinMode(command[1], INPUT);
          Serial.print("Pin "); Serial.print(command[1]); Serial.print(" Value = ");
          Serial.println(digitalRead(command[1]));
          break;

        case 'e': // Analog read
          pinMode(command[1], INPUT);
          Serial.print("Pin "); Serial.print(command[1]); Serial.print(" Value = ");
          Serial.println(analogRead(command[1]));
          break;

        case 'a': // Analog write (PWM)
          pinMode(command[1], OUTPUT);
          analogWrite(command[1], command[2]);
          Serial.print("Writing "); Serial.print(command[2]);
          Serial.print(" to pin "); Serial.println(command[1]);
          break;*/

        default:
          Serial.println("Unknown command.");
          break;
      }
    }
  }
}
