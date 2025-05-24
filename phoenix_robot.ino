#include "phoenix_robot.h"


Phoenix robot(4);

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() { 
  robot.begin();
  //navigator.begin();
  /*
  while !(haishan_koll){
    Serial.println("Whaiting");
    delay(500);
  }
  
  */ 
  while (1){
    if (!digitalRead(4)){break;}
    delay(50);
    Serial.println("Waiting for START...");
  }
  Serial.println("Starting!");
  robot.motgo(-400, -400);
  delay(4000);
  robot.motbrake();
  delay(2000);
  robot.motgo(512, -512);
  delay(4000);
  robot.motbrake();
  delay(2000);
  robot.motgo(-512, 512);
  delay(4000);
  robot.motbrake();
}

void loop() {
  robot.readcli();
  /*
  switch(robot.get_dir()){
    case 'f':
      robot.steer(0,4, 0.1, 2, 400);
      break;
    case 'r':
      robot.gyroturn(90, 350, 4, 2, 0.1);
      break;
    case 'l': 
      robot.gyroturn(-90, 350, 4, 2, 0.1);
      break;
    case 'b':
      robot.gyroturn(180, 350, 4, 2, 0.1);
      break;
    case 'e':
      robot.motbrake();
      break;
  }*/
  Serial.print(robot.get_anglez()); Serial.print("\t"); Serial.println(robot.get_dir());
}  
