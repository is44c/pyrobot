# A bare brain

from pyro.brain import Brain

class SimpleBrain(Brain):
   # Only method you have to define is the step method:

   def __init__(self, robot):
      Brain.__init__(self)
      self.robot = robot

   def step(self):
      #self.robot.move(0, -.2) # negative is to the right!
      print "IR : ----------------------------"
      print "Min distance:", self.robot.getMin().distance
      print "Max distance:", self.robot.getMax().distance

      print "Min angle:", self.robot.getMin().angle
      print "Max angle:", self.robot.getMax().angle
      self.robot.update()

# -------------------------------------------------------
# This is the interface for calling from the gui engine.
# Takes one param (the robot), and returns a Brain object:
# -------------------------------------------------------

def INIT(robot):
   return SimpleBrain(robot)
      