from pyrobot.brain import Brain
from pyrobot.map.navigator import Navigator

class TestNavigator(Brain):

  def setup(self):

     #################################
     # We start with a map built from a to-scale image of the floor-plan
     # of the location of the robot.  Then we localize in that 'perfect'
     # representation of the space.  (The location of the 'real world'
     # marker in the map was determined manually by placing the marker
     # on an easily identifiable feature of the image, like a pillar.)
     #################################
     # cropped map (only the downstairs lab area)
     ################################
     self.nav = Navigator(cols=26,  # approx 40cm each
  		       rows=38, # approx 40cm each
  		       widthMM=float(10383),  # map width in mm
  		       heightMM=float(15184), # map height in mm
     # lab-map-crop-scaled.png has 202x308 pixels
     # map scale information:
     #   xScale 0.0514 (m/pixel), yScale 0.0493 (m/pixel)
  		       image='lab-map-crop-scaled.png',
  		       gridResize = 2)   
     self.nav.placeMarker(row=29,col=6,name="A",angle=0, redraw=1)
     self.nav.placeMarker(row=19,col=13,name="B",angle=3.14, redraw=1)
     self.nav.setGoal(row=4,col=22)

################################
# full map (entire floor of the downstairs lab area)
################################
#     self.nav = Navigator(cols=64,  # approx 40cm each
#  		       rows=124, # approx 40cm
#  		       widthMM=float(25724),  # map width in mm 
#  		       heightMM=float(49537), # map height in mm 
## lab-map-scaled.png has 500x1004 pixels
## map scale information:
##   width 152 pixels = 782cm
##   height 227 pixels = 1120cm
#  		       image='lab-map-scaled.png',
#  		       gridResize = 0.90)   
#     self.nav.placeMarker(row=29,col=6,name="A",angle=0, redraw=1)
#     self.nav.placeMarker(row=19,col=13,name="B",angle=3.14, redraw=1)
#     self.nav.placeMarker(row=37,col=50,name="C",angle=1.57, redraw=1)
#     self.nav.placeMarker(row=49,col=48,name="D",angle=0, redraw=1)
#     self.nav.setGoal(row=4,col=22)
################################

     # then we calculate the robot's current location
     self.nav.updateRobotLocation(self.robot)

     # now we plan a route to the destination
     self.nav.findPath()
     # pop the current location from the list
     self.nav.popPathSubGoal()

  def step(self):

    # first we update the robot's location
    self.nav.updateRobotLocation(self.robot)

    if (self.nav.pathEmpty()):
      print "I am at the goal"
      return 

    print "current Location",self.nav.getCurrentRow(),self.nav.getCurrentCol()

    # if we have already arrived at the next subgoal we remove it and
    # move on to the next.
    if (self.nav.getCurrentSubGoal() == (self.nav.getCurrentRow(),
                                         self.nav.getCurrentCol())):
      print "I reached the old subgoal:",self.nav.getCurrentSubGoal()
      self.nav.popPathSubGoal()

    print "next subgoal",self.nav.getCurrentSubGoal()

    translation, rotate = self.nav.determineMove(self.nav.getCurrentSubGoal(),
                                                 self.robot)
    self.robot.move(translation,rotate)
 
def INIT(engine):
  assert (engine.robot.requires("range-sensor") and
	  engine.robot.requires("continuous-movement"))

  # If we are allowed (for example you can't in a simulation), enable
  # the motors.
  try:
    engine.robot.position[0]._dev.enable(1)
  except AttributeError:
    pass

  return TestNavigator('TestNavigator', engine)
