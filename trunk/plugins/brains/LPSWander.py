# A bare brain with a Local Perceptual View

from pyro.brain import Brain
from random import random
from time import sleep
from pyro.map.lps import LPS
from pyro.map.gps import GPS
from pyro.tools.joystick import Joystick

class SimpleBrain(Brain):
   def setup(self):
      # create the Local Perceptiual Space window
      units = self.getRobot().get('range', 'units')
      self.getRobot().set('range', 'units', 'MM')
      sizeMM = self.getRobot().get('range', 'maxvalue') * 3 + \
               self.getRobot().get('robot', 'radius')
      self.getRobot().set('range', 'units', units)
      self.lps = LPS( 20, 20,
                      widthMM = sizeMM,
                      heightMM = sizeMM)
      self.gps = GPS(100, 100, widthMM = sizeMM * 5, heightMM = sizeMM * 5)
      self.stick = Joystick()
   
   def destroy(self):
      self.lps.destroy()
      self.gps.destroy()
      self.stick.destroy()
   
   def step(self):
      robot = self.getRobot()
      self.lps.reset() # reset counts
      self.lps.sensorHits(robot, 'range')
      self.lps.redraw()
      self.gps.updateFromLPS(self.lps, robot)
      self.getRobot().move( self.stick.translate, self.stick.rotate)
   
# -------------------------------------------------------
# This is the interface for calling from the gui engine.
# Takes one param (the robot), and returns a Brain object:
# -------------------------------------------------------

def INIT(engine):
   return SimpleBrain('SimpleBrain', engine)
      