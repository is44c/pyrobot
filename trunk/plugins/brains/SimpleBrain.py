# A bare brain

from pyro.brain import Brain

class SimpleBrain(Brain):

   def setup(self, **args):
      print args.get('my_arg')
      # initialize your vars here!
      
   # Only method you have to define is the step method:

   def step(self):
      robot = self.getRobot()
      if robot.get('self','stall'):
         print "stuck--reversing"
         robot.move(-0.5, 0)
         sleep(0.5)
      else:
         robot.move(0.5, 0)
      robot.update()
      #self.quit()

# -------------------------------------------------------
# This is the interface for calling from the gui engine.
# Takes one param (an engine), and returns a Brain object:
# -------------------------------------------------------

def INIT(engine):
   return SimpleBrain('SimpleBrain', engine, my_arg = "testing")
      
