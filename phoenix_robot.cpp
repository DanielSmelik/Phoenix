#include "phoenix_robot.h" 
#include "Arduino.h"
//#include "WEMOS_Motor.h"

const unsigned int MAX_MESSAGE_LENGTH = 64;


Phoenix::Phoenix(int test_param){
  _test_param = test_param;
}

//MotorShield MotorShield(MOTORSHIELD_AD11);


void Phoenix::begin(){
  Serial.begin(9600);
  delay(500);
  
  Serial.print("\n");
  Serial.println("Phoenix Robot started...");
  //MotorShield.begin();

  Serial.println("Motors are ON...");
}

void Phoenix::test(int i){
  Serial.print("Testing...   "); Serial.println(i); 
  delay(1000);
}

void Phoenix::motgo(int speed){
  Serial.print("Motors set at speed: "); Serial.print('\t'); Serial.println(speed);
  //MotorShield.drive(speed, speed);  
}

void Phoenix::readcli() {
    char message[MAX_MESSAGE_LENGTH];  // Array to hold the incoming message
    static unsigned int message_pos = 0;  // Keeps track of the position in the message array

    while (Serial.available() > 0) {
        char incomingByte = Serial.read();  // Read the incoming byte from the serial buffer

        // If the incoming byte is not a newline, store it in the message array
        if (incomingByte != '\n') {
            // Ensure the array does not overflow
            if (message_pos < MAX_MESSAGE_LENGTH - 1) {
                message[message_pos] = incomingByte;
                message_pos++;
            } else {
                // Buffer overflow protection: reset the message_pos and discard data
                message_pos = 0;  // Optional: you could choose to discard the message or give feedback
            }
        } else {
            // Null-terminate the message string before printing
            message[message_pos] = '\0';  // Null-terminate the string for proper printing
            Serial.print("You have sent: ");
            Serial.println(message);  // Print the message
            message_pos = 0;  // Reset the position for the next message
        }
    }
}
