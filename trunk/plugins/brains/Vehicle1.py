"""
Braitenberg Vehicle1 for the Khepera
D.S. Blank
"""

from pyro.brain import Brain

class Vehicle(Brain):
   # Only method you have to define for a brain is the step method:

   def step(self):
      maxvalue = 550.0
      sensor = maxvalue -  self.robot.get('light', 'value', 2) # light
      forward = sensor / maxvalue
      self.robot.move(forward,  0.0) # to the left

def INIT(engine):
   if engine.robot.type != 'khepera':
      raise "Robot should have light sensors!"
   print "OK: robot has light sensors."
   return Vehicle('Braitenberg', engine)
      