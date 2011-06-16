import math 

from pyrobot.map.occupancyGrid import OccupancyGrid
from pyrobot.map.mapUtils import getAbsAngle, normalizeAngle2, normalizeAngle, \
                       calculateHypotenuse
from pyrobot.markers.markerToolkit import MarkerToolkit

class Navigator(OccupancyGrid):

  NO_FORWARD = 0
  SLOW_FORWARD = 0.1
  MED_FORWARD = 0.5
  FULL_FORWARD = 1.0

  NO_TURN = 0
  MED_LEFT = 0.5
  HARD_LEFT = 1.0
  MED_RIGHT = -0.5
  HARD_RIGHT = -1.0

  def __init__(self, cols, rows, widthMM, heightMM,
               #the x,y and th of the current location can be passed
               currentRow = -1, currentCol = -1, currentTh= 0,
               goalRow = -1, goalCol = -1, gridUpdate = "",
               image=None, grid=None, gridResize=1.0,showIter=0, title=""):

    self.markerToolkit = MarkerToolkit()

    self.x_localizeMM = 0
    self.y_localizeMM = 0
    self.thr_localize = 0
    self.x_last_marker_localizeMM = 0
    self.x_last_marker_robotMM = 0
    self.y_last_marker_localizeMM = 0
    self.y_last_marker_robotMM = 0
    self.thr_last_marker_localize = 0
    self.thr_last_marker_robot = 0

    self.resetMarkers()

    OccupancyGrid.__init__(self, cols=cols, rows=rows, 
                widthMM=widthMM, heightMM=heightMM,
                currentRow=currentRow, currentCol=currentCol, 
                currentTh=currentTh,
                goalRow=goalRow, goalCol=goalCol,
                image=image, grid=grid, gridResize=gridResize, 
		showIter=showIter, title=title)

  def findPath(self):
    try:
      self.pathList = OccupancyGrid.findPath(self)
    except:
      print "Warning. I wasn't able to find a path"
      self.pathList = []

  def pathEmpty(self):
    try: 
      return (len(self.pathList) == 0)
    except:
      return False

  def getCurrentSubGoal(self):
    return self.pathList[0]

  def popPathSubGoal(self):
    try:
      del self.pathList[0]
    except:
      pass

  def resetMarkers(self):
      self.markers = {}
      self.markerLoc = {}
      self.markerAngle = {}

  # we use the same angle convention used in the pyrobot simulator
  # angles are expressed in radians, 0 is north and + angles rotate
  # counter-clockwise

  def placeMarker(self,row,col,name,angle=0,redraw=0):
      self.markers[name] = (row,col)
      self.markerLoc[(row,col)] = name
      self.markerAngle[name] = angle
      if (redraw):
        self.redrawTrig.set(1)

  def placeRelativeMarker(self,name,relXOffM,relYOffM,relTh,redraw=1):

      # the -1's are to change the perspective.
      newMarkerInfo = self.getPosRelativeToKnown(self.x_localizeMM,#row
                                                 self.y_localizeMM,#col
					         self.thr_localize,
                                                 relXOffM, 
                                                 relYOffM*-1, 
						 relTh*-1)

      print ("placing new marker ",name,newMarkerInfo[0],
             newMarkerInfo[1],newMarkerInfo[4])
             
      self.placeMarker(row=newMarkerInfo[0], col=newMarkerInfo[1],
                       name=name, angle=newMarkerInfo[4], redraw=1)


  def getPosRelativeToKnown(self, knownRowMM, knownColMM, knownAngle, 
                             relXOffM, relYOffM, relTh):

     #print "relXOffM",relXOffM,"relYOffM",relYOffM,"relTh",relTh,\
     #      "atan",math.atan(relYOffM/relXOffM),"knownAngle",knownAngle

     toKnownDistMM = calculateHypotenuse(relXOffM*1000, relYOffM*1000)
     distAbsTh = knownAngle+relTh+math.atan(relYOffM/relXOffM) 
     targetTh = normalizeAngle(knownAngle+relTh+math.pi)

     #print "targetTh",targetTh,"distAbsTh",distAbsTh,"toKnownDistMM", \
     #	   toKnownDistMM,"cos-row",math.cos(distAbsTh),"sin-col", \
     #	   math.sin(distAbsTh)
     #print "rowMMDiff ", (toKnownDistMM * math.cos(distAbsTh)), \
     #      "colMMDiff ", (toKnownDistMM * math.sin(distAbsTh))

     relRowMM = knownRowMM - (toKnownDistMM * math.cos(distAbsTh))
     relColMM = knownColMM - (toKnownDistMM * math.sin(distAbsTh))

     relRow = int(round(relRowMM/self.rowScaleMM))
     relCol = int(round(relColMM/self.colScaleMM))

     return (relRow, relCol, relRowMM, relColMM, targetTh)

  def hasMarker(self,row,col):
      try:
        return self.markerLoc.has_key((row,col))
      except:
        return False 

  def getMarkerRow(self,name):
     return self.markers[name][0]

  def getMarkerCol(self,name):
     return self.markers[name][1]

  def markerExists(self,name):
     return self.markers.has_key(name)

  def printMarker(self,row,col):
     if self.hasMarker(row=row,col=col):
        return "%s-%.2f" % (self.markerLoc[(row,col)],
	                   self.markerAngle[self.markerLoc[(row,col)]])
     else:
        return ""

  def drawCell(self,row,col,matrix,maxval,path,fillCells,removeBelow=0):
  # see the occupancyGrid redraw comment for more info on the coordinate
  # system.

      OccupancyGrid.drawCell(self,row,col,matrix,maxval,path,fillCells,
                             removeBelow)
      x = col
      y = row

      if self.hasMarker(row=row,col=col):
         self.canvas.create_text((x + .5) * self.colScale,
				 (y + .5) * self.rowScale,
				 text=self.printMarker(row=row,col=col),
				 anchor="center",
				 fill="orange",
				 tag = "label")

      try: # there might not be any pathList so we include it in a try
           # maybe it would be better to use __dict__.has_key here?
        if self.pathList.index((y,x)) >= 0:
           self.canvas.create_rectangle((x + .4) * self.colScale,
					(y + .4) * self.rowScale,
					(x + .6) * self.colScale,
					(y + .6) * self.rowScale,
					width = 0,
					fill = "blue",
					tag = "path")
      except:
        pass

  def setRobotLocation(self,robotXMM,robotYMM,robotTh):

    self.x_localizeMM = robotXMM
    self.y_localizeMM = robotYMM
    self.thr_localize = robotTh

    self.setCurrent(row=int(round(self.x_localizeMM/self.rowScaleMM)),
                      col=int(round(self.y_localizeMM/self.colScaleMM)),
                      th=self.thr_localize)

  def updateRobotLocation(self,robot):
    # get the list of markers in view
    markers = self.markerToolkit.getMarkerData(robot)
    # print "I found markers: ",markers 
   
    # if we can see at least one marker 
    if (isinstance(markers, list) and len(markers) >= 1):

       # print "localizing using seen markers"
       # we look for the closest marker of those found, we will localize
       # upon that (assuming less error the closer the marker)
       closestMarker = -1
       for index in range(len(markers)):
          if (self.markerExists(markers[index][3]) and
              (closestMarker == -1 or
               # we assume that negative distance markers are errors
               (markers[closestMarker][0] > markers[index][0] and
                markers[index][0] > 0))):
            closestMarker = index

       if (closestMarker >= 0):
          # print "Localizing using closest marker: ",markers[closestMarker]
   
	  # localize sets the current position and orientation of the robot in
	  # the grid and returns that position as well as more precise MM based
	  # location information which we will use internally.  (remember x=row
	  # y=col)
          coord_robot = self.localize(markerName=markers[closestMarker][3],
                                        relXOffM=markers[closestMarker][0],
                                        relYOffM=markers[closestMarker][1],
                                        relTh=markers[closestMarker][2])
   
	  # print "Localization diff: x",(self.x_localizeMM-coord_robot[0]),\
	  #                       "y",(self.y_localizeMM-coord_robot[1]),\
	  #			 "thr-old",self.thr_localize,\
	  #			 "thr-new",coord_robot[2]

          self.x_localizeMM = coord_robot[0]
          self.y_localizeMM = coord_robot[1]
	  self.thr_localize = coord_robot[2]

          # temporarily change the robot's unit system to meters
	  # it seems as though a simulated robot doesn't have a units
	  # attribute??
	  try:
	    oldUnits = robot.units
	  except AttributeError:
	    oldUnits = "METERS"
          robot.units = "METERS"

          # We keep track of where we think that we are and where the robot
          # thinks that it is to be able to localize without markers
          # in future passes.
          self.x_last_marker_localizeMM = self.x_localizeMM
          self.x_last_marker_robotMM = robot.x*1000
          self.y_last_marker_localizeMM = self.y_localizeMM 
          self.y_last_marker_robotMM = robot.y*1000
          self.thr_last_marker_localize = self.thr_localize
          self.thr_last_marker_robot = robot.thr

          robot.units = oldUnits

       # Lastly, we look through all the seen markers and if there are
       # any that are NEW we place them in the map (remember that we
       # can't localize upon a new marker that is why this comes at the
       # end) Note, if we start out with an empty map this will
       # place the marker based upon the assumed starting 'current'
       # location.

       for index in range(len(markers)):
          if (not self.markerExists(markers[index][3]) and
              not markers[index][3] == ""):
            self.placeRelativeMarker(name=markers[index][3],
                                       relXOffM=markers[index][0],
                                       relYOffM=markers[index][1],
				       relTh=markers[index][2])

    # otherwise we localize using odometry
    else: 
	# print "Localizing with odometry"
        # temporarily change the robot's unit system to meters
	# it seems as though a simulated robot doesn't have a units
	# attribute so we try and if not there we just add it.
	try:
	  oldUnits = robot.units
	except AttributeError:
	  oldUnits = "METERS"
        robot.units = "METERS"

        xDiffMM = (robot.x*1000) - self.x_last_marker_robotMM
        yDiffMM = (robot.y*1000) - self.y_last_marker_robotMM
        thDiff = self.thr_last_marker_robot - self.thr_last_marker_localize
	# print "diffs",xDiffMM,yDiffMM,thDiff

        # we need to remember that the robot's odometry was set when the 
	# robot started and thus its coordinate system could be very different,
	# most likely even rotated with respect to the map's coordinates.
        # also there is a sign difference between the map's Y and the i
        # y given by the odometry.

    	self.x_localizeMM = (self.x_last_marker_localizeMM -
                             (math.cos(thDiff)*xDiffMM + 
                              -1*math.sin(thDiff)*yDiffMM))
	self.y_localizeMM = (self.y_last_marker_localizeMM +
                             math.sin(thDiff)*xDiffMM +
                             -1*math.cos(thDiff)*yDiffMM)
	self.thr_localize = normalizeAngle(robot.thr - 
					   self.thr_last_marker_robot + 
					   self.thr_last_marker_localize)
        # print "new location",self.x_localizeMM,self.y_localizeMM,self.thr_localize

        self.setCurrent(row=int(round(self.x_localizeMM/self.rowScaleMM)),
                          col=int(round(self.y_localizeMM/self.colScaleMM)),
                          th=self.thr_localize)

        robot.units = oldUnits

  def localize(self, markerName, relXOffM, relYOffM, relTh):

   # We are localizing using a marker in the field of vision of the robot.
   # relXOffM, relYOffM are the legs of the triangle made from the straight
   # line from the robot to the marker (with the base of the right triangle
   # along the marker + relTh).  relXOffM is along the 'heading' line' and
   # relYOffM is at 90 degrees to the heading line

     try:
       # assume that the marker is in the center of the cell. (I have
       # left this as .45 to prevent it from rounding 'up' in other
       # places which causes problems with alignemnt.
       markerRowMM = (self.getMarkerRow(markerName)+.45)*self.rowScaleMM
       markerColMM = (self.getMarkerCol(markerName)+.45)*self.colScaleMM
       markerAngle = self.markerAngle[markerName]
     except KeyError:
       # we don't have info regarding that marker??
       return (-1,-1,-1)

     #the return of getPosRelativeToKnown is (row,col,rowMM,colMM,th) 
     coord_robot = self.getPosRelativeToKnown(markerRowMM,
                                              markerColMM,
					      markerAngle,
                                              relXOffM, 
                                              relYOffM, relTh)

     #print " marker - row ",self.getMarkerRow(markerName)," col ",\
     #      self.getMarkerCol(markerName)," th ", markerAngle
     #print "        - rowMM ",markerRowMM," colMM ", markerColMM
     #print " robot - row ",coord_robot[0]," col ",coord_robot[1]," th ", \
     #      coord_robot[4]
     #print "        - rowMM ",coord_robot[2]," colMM ", coord_robot[3] 

     self.setCurrent(row=coord_robot[0],col=coord_robot[1],
                     th=coord_robot[4])
     self.redrawTrig.set(1)

     # return the location in MM and the th
     return (coord_robot[2], coord_robot[3], coord_robot[4])

  def determineMove(self, subgoal, robot=None):
     # note that we assume that the row and col given are subgoals, this
     # algorithm can NOT deal with walls or obstacles.

     subgoalRow, subgoalCol = subgoal

     # the 0.5 is to get the robot to move towards the CENTER of that
     # row and col cell.
     subgoalXmm = (subgoalRow+0.5)*self.rowScaleMM
     subgoalYmm = (subgoalCol+0.5)*self.colScaleMM
     
     # remember that in pyrobot th increases as the robot turns
     # COUNTER-CLOCKWISE

     # we start by calculating the difference between the current
     # heading (orientation angle) of the robot and a straight line to
     # the subgoal
     angleToGoal = math.atan2((subgoalYmm-self.y_localizeMM)*-1,
                              (subgoalXmm-self.x_localizeMM)*-1)
                             
     robotAngleDiff = normalizeAngle(angleToGoal-self.thr_localize)

     #print "Current: ",self.x_localizeMM, self.y_localizeMM, self.thr_localize
     #print "Subgoal: ",subgoalXmm, subgoalYmm
     #print "angleToGoal",angleToGoal
     #print "robotAngleDiff: ",robotAngleDiff

     # if the subgoal isn't within an arc of size pi/4 centered in the
     # front of the robot we do a hard turn
     if (robotAngleDiff > math.pi/8 and robotAngleDiff <= math.pi):
        print "hard left"
        return(self.NO_FORWARD, self.HARD_LEFT)
     elif (robotAngleDiff > math.pi and 
           robotAngleDiff < (math.pi*2 - (math.pi/8))):
        print "hard right"
        return(self.NO_FORWARD, self.HARD_RIGHT)

     # when within the pi/4 arc we turn less the closer we are to
     # having it straight ahead
     elif (robotAngleDiff > 0 and robotAngleDiff <= math.pi/8):
        print "med-forward, variable to the left"
        return(self.MED_FORWARD, robotAngleDiff/math.pi/8)
     elif (robotAngleDiff >= (math.pi*2 - (math.pi/8)) and
           robotAngleDiff < math.pi*2):
        print "med-forward, variable to the right"
        return(self.MED_FORWARD, (robotAngleDiff-math.pi*2)/math.pi/8)

     # if it is straight ahead to just go straight
     else:
        print "forward"
        return(self.FULL_FORWARD,self.NO_TURN)

  """
  This method extracts all the information from another grid to include
  it in this grid.  It starts by looking for two common markers in this
  and the other grid.  If 2 can not be found it fails, otherwise it
  uses these markers to calculate the transformation between the grids
  and then combines the information using this transformation and the
  grid update algorithms.
  """
  def mergeOtherGridInfo(self, otherGrid):

      # find two common markers
      commonKeys = []
      for key in self.markers.keys():
        if (otherGrid.markerExists(key)):
          commonKeys.append(key)

      if (len(commonKeys) < 2):
        print "I can not find two common markers, merge failed!"
        return

      # for now we just use the first two common markers found
      thisGridAnchor1Row = self.getMarkerRow(commonKeys[0])
      thisGridAnchor1Col = self.getMarkerCol(commonKeys[0])
      thisGridAnchor2Row = self.getMarkerRow(commonKeys[1])
      thisGridAnchor2Col = self.getMarkerCol(commonKeys[1])
      otherGridAnchor1Row = otherGrid.getMarkerRow(commonKeys[0]) 
      otherGridAnchor1Col = otherGrid.getMarkerCol(commonKeys[0])
      otherGridAnchor2Row = otherGrid.getMarkerRow(commonKeys[1])
      otherGridAnchor2Col = otherGrid.getMarkerCol(commonKeys[1])

      return OccupancyGrid.mergeOtherGridInfo(self,otherGrid,
                                     thisGridAnchor1Row = thisGridAnchor1Row,
				     thisGridAnchor1Col = thisGridAnchor1Col,
				     thisGridAnchor2Row = thisGridAnchor2Row,
				     thisGridAnchor2Col = thisGridAnchor2Col,
				     otherGridAnchor1Row = otherGridAnchor1Row,
				     otherGridAnchor1Col = otherGridAnchor1Col,
				     otherGridAnchor2Row = otherGridAnchor2Row,
				     otherGridAnchor2Col = otherGridAnchor2Col)
