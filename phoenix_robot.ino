#include "phoenix_robot.h"
//#include "clicli.h"


Phoenix robot(1);
//clicli mycli(robot);  

int i = 0;

void setup() { 
  robot.begin();
  //mycli.begin();

}

void loop() {
  //mycli.run();
  //robot.test(i);
  //i++;
  robot.motgo(100); 
  delay(3000);
  robot.motgo(0);
  delay(3000);
  }
