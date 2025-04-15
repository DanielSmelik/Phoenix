#include "phoenix_robot.h" 
#include "Arduino.h"
#include "WEMOS_Motor.h"

const unsigned int MAX_MESSAGE_LENGTH = 64;


Phoenix::Phoenix(int _test_arg){
  int test_arg = _test_arg;
}

int flamePins[] = {3, 5, 6, 9, 10}; 
const int numFlameSensors = 5;
const int flameThreshold = 2000;

MotorShield MotorShield(MOTORSHIELD_AD11);

void Phoenix::begin(){
  Serial.begin(9600);
  delay(500);
  
  Serial.print("\n");
  Serial.println("Phoenix Robot started...");
  MotorShield.begin();
  delay(1000);
  Serial.println("Motors are ON...");
  
  Serial.println("Flame sensors are ON...");
  }

void Phoenix::motgo(int speedl, int speedr){
  Serial.print("Motors set at speed: "); Serial.print('\t'); Serial.print(speedl); Serial.print('\t'); Serial.println(speedr);
  MotorShield.drive(speedl, speedr);  
}

bool Phoenix::detectFlame(int values[]) {
    bool flameDetected = false;

    Serial.print("Flame detected by sensor(s): ");
    for (int i = 0; i < numFlameSensors; i++) {
        if (values[i] > flameThreshold) {
            Serial.print(i + 1); // Sensor numbers are 1-based
            Serial.print(" ");
            flameDetected = true;
        }
    }

    if (flameDetected) {
        Serial.println(); // Move to the next line
    } else {
        Serial.println("None");
    }

    return flameDetected;
}

void Phoenix::readcli() {
     while (Serial.available() > 0) { 
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;

    char inByte = Serial.read();

    // Build message until newline or max length reached
    if (inByte != '\n' && message_pos < MAX_MESSAGE_LENGTH - 1) {
      message[message_pos++] = inByte;
    } else {
      // End of message
      message[message_pos] = '\0'; // Null-terminate
      Serial.println(message);     // Echo the message

      // Reset position for next message
      message_pos = 0;

      // Temporary copy for parsing
      char tmpmsg[MAX_MESSAGE_LENGTH];
      strcpy(tmpmsg, message);

      char *ptr = strtok(tmpmsg, " ");
      char cmd = '\0';
      int command[4] = {0};
      int argindex = 0;

      // Tokenize and parse arguments
      while (ptr != NULL && argindex < 4) {
        if (argindex == 0) {
          cmd = ptr[0]; // First token is command character
        } else {
          command[argindex] = atoi(ptr); // Convert argument to int
        }
        argindex++;
        ptr = strtok(NULL, " ");
      }

      // Execute command
      switch (cmd) {
        case 'h': // Set pin HIGH
          pinMode(command[1], OUTPUT);
          digitalWrite(command[1], HIGH);
          Serial.print("Pin "); Serial.print(command[1]); Serial.println(" is SET");
          break;

        case 'l': // Set pin LOW
          pinMode(command[1], OUTPUT);
          digitalWrite(command[1], LOW);
          Serial.print("Pin "); Serial.print(command[1]); Serial.println(" is RESET");
          break;

        case 'm': //  Both motors control at given speed.
          motgo(command[1], command[1]);
          break;

        case 'R': //  turn right at speed.
          motgo(0, command[1]);
          break;
        case 'L':
          motgo(command[1], 0);
          break;
          
        case 'd': // Digital read
          pinMode(command[1], INPUT);
          Serial.print("Pin "); Serial.print(command[1]); Serial.print(" Value = ");
          Serial.println(digitalRead(command[1]));
          break;

        case 'e': // Analog read
          pinMode(command[1], INPUT);
          Serial.print("Pin "); Serial.print(command[1]); Serial.print(" Value = ");
          Serial.println(analogRead(command[1]));
          break;

        case 'a': // Analog write (PWM)
          pinMode(command[1], OUTPUT);
          analogWrite(command[1], command[2]);
          Serial.print("Writing "); Serial.print(command[2]);
          Serial.print(" to pin "); Serial.println(command[1]);
          break;

        default:
          Serial.println("Unknown command.");
          break;
      }
    }
  }
}
