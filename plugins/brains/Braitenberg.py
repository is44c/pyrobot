"""
A Braitenberg Vehicle for the Khepera
D.S. Blank
"""

# import some additional libraries:

from pyro.brain import Brain
from math import fabs

# subclass brain:

class Vehicle(Brain):
   # Only method you have to define for a brain is the step method:

   def step(self):
       left = self.robot.get('/robot/light/1/value')  # Khepera specific
       right = self.robot.get('/robot/light/4/value') # Khepera specific

       turn = (right - left) / max(self.robot.get('/robot/light/all/value'))
       forward = .3 - fabs(turn * .3) # go when no light

       self.robot.move(forward, turn)

# The function INIT that takes a robot and returns a brain:

def INIT(engine):
   if engine.robot.get('/robot/type') != 'khepera':
      raise "Robot should have light sensors!"
   print "OK: robot has light sensors."
   return Vehicle('Braitenberg', engine)
      