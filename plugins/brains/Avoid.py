# robot goes forward and then slows to a stop when it detects something  
   
from pyro.brain import Brain  
from pyro.brain.conx import *  
from time import *  
   
class Avoid(Brain):  
           
   # Give the front two sensors, decide the next move  
   def determineMove(self, front, left, right):  
      if front < 0.5:   # about to hit soon, STOP  
         #print "collision imminent, stopped"  
         return(0, .3)  
      elif left < 0.8:
         return(0.1, -.3)  
      elif right < 0.8:  # detecting something, SLOWDOWN  
         #print "object detected"  
         return(0.1, .3)  
      else:  
         #print "clear"      # all clear, FORWARD  
         return(0.3, 0.0) 
      
   def step(self):  
      r = self.robot
      front = min(r.get('/robot/range/front/value'))
      left = min(r.get('/robot/range/left-front/value'))
      right = min(r.get('/robot/range/right-front/value'))
      translation, rotate = self.determineMove(front, left, right)  
      robot.move(translation, rotate)  
         
def INIT(robot):  
   return Avoid('Avoid', robot)  
