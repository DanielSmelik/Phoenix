#include "phoenix_robot.h" 
#include "Arduino.h"

Phoenix::Phoenix(int test_param){
  _test_param = test_param;
}

void Phoenix::begin(){
  Serial.begin(115200);
  Serial.println("Phoenix Robot initialization started...");
}

void Phoenix::motgo(int speed){
  Serial.print("Motors set at speed: "); Serial.print('\t'); Serial.println(speed);
}