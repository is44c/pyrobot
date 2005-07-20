# robot goes forward and then slows to a stop when it detects something  
   
from pyrobot.brain import Brain  
from pyrobot.brain.conx import *  
from time import *  
   
class Avoid(Brain):  
           
   # Give the front two sensors, decide the next move  
   def determineMove(self, front, left, right):  
      if front < 1.5:   # about to hit soon, STOP  
         #print "collision imminent, stopped"  
         return(0, .3)  
      elif left < 1.0:
         return(0.1, -.3)  
      elif right < 1.0:  # detecting something, SLOWDOWN  
         #print "object detected"  
         return(0.1, .3)  
      else:  
         #print "clear"      # all clear, FORWARD  
         return(0.5, 0.0) 
      
   def step(self):  
      front = min([s.distance() for s in self.robot.range["front"]])
      left = min([s.distance() for s in self.robot.range["left-front"]])
      right = min([s.distance() for s in self.robot.range["right-front"]])
      translation, rotate = self.determineMove(front, left, right)  
      self.robot.move(translation, rotate)

def INIT(engine):  
   assert (engine.robot.requires("range-sensor") and
           engine.robot.requires("continuous-movement"))
   return Avoid('Avoid', engine)  

