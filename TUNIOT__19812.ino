#include <Arduino.h>

// --- Pin Configuration ---
const int flameSensorPins[] = {25, 26, 27, 14, 13}; // GPIO pins for flame sensors
const int numFlameSensors = sizeof(flameSensorPins) / sizeof(flameSensorPins[0]);
const int flameThreshold = 2000; // Threshold for detecting a flame (adjust based on your sensor)

// Variables to track time
unsigned long startTime;
const unsigned long duration = 60000; // 1 minute = 60000 milliseconds

// Function to read values from all sensors
void readFlameSensors(int values[]) {
    for (int i = 0; i < numFlameSensors; i++) {
        values[i] = analogRead(flameSensorPins[i]);
        Serial.print(values[i]);
    }
}

// Function to check if any sensor detects a flame and print the sensor number(s)
bool detectFlame(int values[]) {
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
const int motorIA = 32; // Connect to IA on L9110
const int motorIB = 33; // Connect to IB on L9110
void setup() {
    Serial.begin(9600);
    pinMode(motorIA, OUTPUT);
    pinMode(motorIB, OUTPUT); 
    // Initialize flame sensor pins as inputs
    for (int i = 0; i < numFlameSensors; i++) {
        pinMode(flameSensorPins[i], INPUT);
    }

    Serial.println("5-channel flame sensor module initialized.");

    // Record the start time
    startTime = millis();
}
void stop_fan(){
    digitalWrite(motorIA, LOW);
    digitalWrite(motorIB, LOW);
}
void run_fan(){
 // Set motor to rotate clockwise
  analogWrite(motorIA, 225); // Full speed
  digitalWrite(motorIB, LOW);
  delay(4000);

  
  //delay(5000); // Run for 5 seconds

  // Set motor to rotate anticlockwise
  //digitalWrite(motorIA, LOW);
 // analogWrite(motorIB, 255); // Full speed
 // delay(2000); // Run for 2 seconds

  // Stop the motor
  //digitalWrite(motorIA, LOW);
  //digitalWrite(motorIB, LOW);
  //delay(1000); // Stop for 1 second*/

}        
void loop(){
  int sensorValues[numFlameSensors];
  readFlameSensors(sensorValues);
  for (int i = 0; i < numFlameSensors; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
}
/*void loop() {
    // Check if 1 minute has passed
    if (millis() - startTime < duration) {
        int sensorValues[numFlameSensors];

        // Read sensor values
        readFlameSensors(sensorValues);

        // Print sensor values to the Serial Monitor
        Serial.print("Sensor values: ");
        for (int i = 0; i < numFlameSensors; i++) {
            Serial.print(sensorValues[i]);
            Serial.print(" ");
        }
        Serial.println();

        // Check if any sensor detects a flame
        if (detectFlame(sensorValues)) {
            Serial.println("Flame detected!");
            run_fan();
            delay(4000);
        } else {
            Serial.println("No flame detected.");
            stop_fan();
        }
        
        delay(1000); // Small delay to avoid flooding the Serial Monitor
    } else {
        // After 1 minute, print that the check is complete
        Serial.println("Flame detection complete. 1 minute has passed.");
        // Optional: You could stop the loop or put the ESP32 into a low-power mode here
        while (true);  // Stop further execution
    }
    stop_fan();
}
*/