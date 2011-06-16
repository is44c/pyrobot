# A bare brain with a Local Perceptual View

from pyrobot.brain import Brain
from pyrobot.map.lps import LPS
from pyrobot.map.gps import GPS
from pyrobot.tools.joystick import Joystick
import thread

class JoystickMapping(Brain):
   def setup(self):
      # we want to map in MM
      units = self.robot.range.units
      self.robot.range.units = 'MM'
      sizeMM = self.robot.range.getMaxvalue() * 2 + \
               (self.robot.radius / 1000.0)
      self.robot.range.units = units
      # create the Local Perceptiual Space window to hold the local map
      self.lps = LPS( 80, 80,
                      widthMM = sizeMM,
                      heightMM = sizeMM)
      # create the Global Perceptiual Space window to hold the global map
      self.gps = GPS(100, 100, widthMM = float(sizeMM), 
                     heightMM = float(sizeMM))
      self.stick = Joystick()
      self.needCompleteRedraw = True
      self.needRedraw = False
      self.lock = thread.allocate_lock()
   
   def destroy(self):
      self.lps.destroy()
      self.gps.destroy()
      self.stick.destroy()
   
   def step(self):
      if not self.lock.acquire(False):
	 return
      self.lps.reset() # reset counts
      self.lps.sensorHits(self.robot, 'range')
      self.gps.updateFromLPS(self.lps, self.robot)
      self.needRedraw = True
      self.robot.move( self.stick.translate, self.stick.rotate)
      self.lock.release()

   def redraw(self):
      if (not self.lock.acquire(False)):
	return
      if self.needRedraw:
	self.lps.redraw(drawLabels=False)
        self.gps.update()
	if self.needCompleteRedraw:
	  self.gps.redraw()
	  self.needCompleteRedraw = False
	else:
	  self.gps.redraw(onlyDirty=1)
	self.needRedraw = False
      self.lock.release()
   
# -------------------------------------------------------
# This is the interface for calling from the gui engine.
# Takes one param (the robot), and returns a Brain object:
# -------------------------------------------------------

def INIT(engine):
   return JoystickMapping('JoystickMapping', engine)
