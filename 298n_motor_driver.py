"""This library manages the L298N motor driver. For MicroPython.  

The Motor class 3 values:
digital_pinA, digital_pinB, analog_pin. All int type. 
The Motor class has the following methods: 
turn_to(dir, speed)
drive(dir, speed)
stop()

Written by Daniel Smelik, 11th grade, 4th class, Gvanim Robotics Lab, 2025.01.08. 
Feel free to use this code in all your projects. 
"""
from machine import Pin, PWM
#from numpy import interp
import time 

class Motor():
  def __init__(self, digital_pinA, digital_pinB, analog_pin):
    self.digitalA = Pin(digital_pinA, Pin.OUT)
    self.digitalB = Pin(digital_pinB, Pin.OUT)
    self.analog = PWM(Pin(analog_pin))
  
  def drive(self, dir, speed):
    #dir should be "forward" or "backward". speed between 0 and 100. Forward is defined as the first pin being high, the second being low. 
    #mapped_speed = interp(speed, [0,100], [0, 1023]) # mapping the speed int for PWM values. 
    
    if dir == "forward":
      self.digitalA.on()
      self.digitalB.off()
      self.analog.duty(speed)
    print("Going Forward.")
      
