"""
Braitenberg Vehicle2b
The more light sensed on the left side the faster the right motor moves.
The more light sensed on the right side the faster the left motor moves.
This causes the robot to turn towards a light source.
"""

from pyrobot.brain import Brain, avg

class Vehicle(Brain):
   def setup(self):
      self.robot.light[0].units = "SCALED"
   def step(self):
      leftSpeed  = self.robot.light[0]["right"][0].value 
      rightSpeed = self.robot.light[0]["left"][0].value
      print "leftSpeed, rightSpeed:", leftSpeed, rightSpeed
      self.motors(leftSpeed,  rightSpeed) 

def INIT(engine):
   if engine.robot.type not in ['K-Team', 'Pyrobot']:
      raise "Robot should have light sensors!"
   return Vehicle('Braitenberg2a', engine)
      
