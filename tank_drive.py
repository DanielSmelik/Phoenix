import time
from neopixel import NeoPixel 
from PhoenixRobot import PhoenixRobot as Robot

# Assuming the Robot class is defined elsewhere in your code

class TankDriveVehicle(Robot): # this class is inherited from a "Robot" class. A class that has: self.motorA and self.motorB, which are Motor class objects. 
  # A Tank Drive Vehicle is a Vehicle with 2 motors, without front wheel steering. This class gets 2 lists, where the first 2 elements are the digital pins and the last is the analog one.  
  def super()__init__(self):
    self.motorA = Robot.motorA # Assuming Robot initializes motorA
    self.motorB = Robot.motorB # Assuming Robot initializes motorB

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
