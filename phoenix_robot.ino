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
  robot.motgo(400, 400);
}

void loop() {
  //static bool lastButtonState = HIGH;
  //bool currentButtonState = digitalRead(4);

  robot.readcli();
  /*
  if (lastButtonState != currentButtonState) {
    lastDebounceTime = millis(); // reset the debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentButtonState == LOW) { // button is pressed
      robot.motgo(300, 300);
    }
  }

  lastButtonState = currentButtonState;*/
  Serial.print(robot.get_anglez()); Serial.print("\t"); Serial.println(robot.get_dir());
}  
