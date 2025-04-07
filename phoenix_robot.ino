#include "phoenix_robot.h"
#include "navigator.h"

Phoenix robot(1);
navigator navigator(1,2,3,4,5,6,7);

int i = 0;

void setup() { 
  Serial.begin(115200);
  robot.begin();
  navigator.begin();

}

void loop() {
  //robot.test(i);
  //i++;
  /*robot.motgo(100); 
  delay(3000);
  robot.motgo(0);
  delay(3000);
  */
  robot.readcli();
  delay(100);
  navigator.run();
  }



