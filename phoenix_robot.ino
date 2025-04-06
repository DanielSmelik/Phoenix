#include "phoenix_robot.h"


Phoenix robot(1);

int i = 0;

void setup() { 
  robot.begin();

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
  }
