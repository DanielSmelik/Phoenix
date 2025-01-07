"""Phoenix Project by Daniel Smelik, Tamat Grimberg and Yuval Rachman. Winter 2025. 
Phoenix Project is the code name of our robot for the ROBONER and the Nationonal Robotics
and Engineering Competion. 
Work on this project began in november of 2024.

Work on this program began on Jan 7 2025. 
"""

#from YuvalsClass import navigaion_func
#from TamarsClass import search_for_fire, extinguish_fire
#import LEDS. 
import math 
import json 
import time 

print("Hello World, from Phoenix!")

class PhoenixRobot():
  #Main class of the Phoenix project. 
  def __init__(self): 
    pass

  def main_loop(self):
    while True:
      print("Searching for Fire.")
      time.sleep(0.25)
      #navigate(kp, ki, kd)
      #if search_for_fire(): extinguish_fire()

  def load_settings(self):
    #this func loads setting from json file.  
    pass
  def on_close(self):
    #this func is the last to run when the robot is turned off. It sends a message and dumps setting into the json.
    pass

if __name__ == "__main__":
  phoenix = PhoenixRobot()
  phoenix.main_loop()
