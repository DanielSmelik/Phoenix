#include <Adafruit_NeoPixel.h>

// Pin where NeoPixel data input is connected
#define PIN            5    // Pin 5 on Arduino for NeoPixel data
// Number of LEDs in the NeoPixel ring
#define NUMPIXELS      12   // Set the correct number for your NeoPixel ring
int sum = 0;
// Initialize the NeoPixel object
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // Initialize the NeoPixel ring
  strip.begin();
  strip.show();  // Ensure all LEDs are turned off initially
}

void loop() {
  circulatingEffect();  // Call the circulating LED effect
  delay(100);  // Delay to control speed of effect
}

// Function to create circulating LEDs effect
void circulatingEffect() {
  // Turn off all pixels first
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));  // Set all LEDs to off
  }
  
  // Create a circulating effect by lighting up each pixel one by one
  for (int i = 0; i < strip.numPixels(); i++) {
    if (sum>5)
      sum = 0;
    // Light up the current LED (red color in this case)
    strip.setPixelColor(sum, strip.Color(255, 4, 12));// Red color
    strip.setPixelColor(sum+1, strip.Color(250, 76, 46));// Red color
    strip.setPixelColor(sum+2, strip.Color(49, 22, 90));// Red color
    strip.setPixelColor(sum+3, strip.Color(57, 11, 138));// Red color
    strip.setPixelColor(sum+4, strip.Color(34, 13, 235));// Red color
    strip.setPixelColor(sum+5, strip.Color(13, 65, 245));// Red color
    strip.setPixelColor(sum+6, strip.Color(76, 90, 45));// Red color

 
    
    strip.show();  // Update the NeoPixel ring
    delay(100);  // Delay to control speed of movement

    // Turn off the LED after the delay
    strip.setPixelColor(sum, strip.Color(0, 0, 0));  // Set it to off
    strip.setPixelColor(sum+1, strip.Color(0, 0, 0));  // Set it to off
    strip.setPixelColor(sum+2, strip.Color(0, 0, 0));  // Set it to off
    strip.setPixelColor(sum+3, strip.Color(0, 0, 0));  // Set it to off
    strip.setPixelColor(sum+4, strip.Color(0, 0, 0));  // Set it to off
    strip.setPixelColor(sum+5, strip.Color(0, 0, 0));  // Set it to off
    strip.setPixelColor(sum+6, strip.Color(0, 0, 0));  // Set it to off

    sum++;
   
  }
}

