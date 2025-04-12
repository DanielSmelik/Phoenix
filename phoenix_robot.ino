#include "phoenix_robot.h"
#include "navigator.h"

int flamePins[] = {3, 5, 6, 9, 10};  
Phoenix robot(flamePins,5, 2000);
navigator navigator(robot, 1,2,3,4,5,6, 7);  

int i = 0;

void setup() { 
  robot.begin();
  /*
  while !(haishan_koll){
    Serial.println("Whaiting");
    delay(500);
  }
  */ 
}

void loop() {
  robot.readcli();
  }
