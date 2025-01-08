"""A "Tank" driving system for vehicles with two motors. By "tank drive," I mean a driving system where,
to make a turn, the wheels or tracks spin in opposite directions. 
By Daniel Smelik, Gvanim Robotics Lab, 2025.01.08.
"""

import time
from neopixel import NeoPixel 
from l298n_motor_driver import motor  

class TankDriveVehicle():
  # A Tank Drive Vehicle is a Vehicle with 2 motors, without front wheel steering. This class gets 2 lists, where the first 2 elements are the digital pins and the last is the analog one.  
  def __init__(self, motor1_pins, motor2_pins):
    self.motorA = motor(motor1_pins[0], motor1_pins[1], motor1_pinds[2])
    self.motorB = motor(motor2_pins[0], motor2_pins[1], motor2_pinds[2)

  def go_forward(self, speed):
    self.motorA.run("forward", speed)
    self.motorB.run("forward", speed)
  def go_backward(self, speed):
    self.motorA.run("backward", speed)
    self.motorB.run("backward", speed)
  def go_right(self, speed):
    self.motorA.run("forward", speed)
    self.motorB.run("backward", speed)
  def go_right(self, speed):
    self.motorA.run("backward", speed)
    self.motorB.run("forward", speed)
      
    
