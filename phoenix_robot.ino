#include "phoenix_robot.h"
#include "navigator.h"

 
Phoenix robot(5);
navigator navigator(robot, 1,2,3,4,5,6, 7);  

int i = 0;

void setup() { 
  robot.begin();
  navigator.begin();
  /*
  while !(haishan_koll){
    Serial.println("Whaiting");
    delay(500);
  }
  
  */ 
}

void loop() {
  robot.readcli();
  /*
  navigator.go_to_dir();
  if (robot.detectFlame(int values[] ???)){
    robot.stop(); or robor.motgo(0,0);
    robot.killFlame();
  }
  */
  }
