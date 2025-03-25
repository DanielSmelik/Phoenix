#include "clicli.h"
#include "phoenix_robot.h"

Phoenix robot(1);
clicli mycli(robot);  

void setup() { 
  
  mycli.begin();
  

 }

void loop() { 
  mycli.run();
 }
