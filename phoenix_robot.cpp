#include "phoenix_robot.h" 
#include "Arduino.h"
//#include <iarduino_I2C_Motor.h>              
#include "WEMOS_Motor.h"


Phoenix::Phoenix(int test_param){
  _test_param = test_param;
}

//iarduino_I2C_Motor mot_left(0x09); //left/right will change
MotorShield MotorShield(MOTORSHIELD_AD11);


void Phoenix::begin(){
  Serial.begin(9600);
  delay(500);
  
  Serial.print("\n");
  Serial.println("Phoenix Robot started...");
  MotorShield.begin();

  Serial.println("Motors are ON...");
}

void Phoenix::test(int i){
  Serial.print("Testing...   "); Serial.println(i); 
  delay(1000);
}

void Phoenix::motgo(int speed){
  Serial.print("Motors set at speed: "); Serial.print('\t'); Serial.println(speed);
  MotorShield.drive(speed, speed);  
}
