"""A "Tank" driving system for vehicles with two motors. By "tank drive," I mean a driving system where,
to make a turn, the wheels or tracks spin in opposite directions. 
By Daniel Smelik, Gvanim Robotics Lab, 2025.01.08.
"""

import time
from neopixel import NeoPixel 
#from ln298_motor_driver import motor  

class TankDriveVehicle(motor1_pins, motor2_pins):
  # A Tank Drive Vehicle is a Vehicle with 2 motors, without front wheel steering. This class gets 2 lists, where the first 2 elements are the digital pins and the last is the analog one.  
  def __init__(self):
    motorA = motor(motor1_pins)
    motorB = motor(motor2_pins)
  
    
