#include "clicli.h"
#include "phoenix_robot.h"
#include "navigator.h" 

Phoenix robot(1);
clicli mycli(robot);  
navigator navigator(0);

void setup() { 
  
  mycli.begin();
  robot.begin();
  navigator.begin();
  navigator.gyro_value();
  


 }

void loop() { 
  mycli.run();
 }
