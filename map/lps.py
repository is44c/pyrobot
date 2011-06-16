from pyrobot.map.tkmap import TkMap
import math

class LPS(TkMap):
   """
   GUI for visualizing the local perceptual space of a robot.
   """
   def __init__(self, cols, rows, 
                width = 200, height = 200,
                widthMM = 7500, heightMM = 7500, 
                title = "Local Perceptual Space"):
      """ Pass in grid cols, grid cells, and total width/height in MM """

      self.newCellLabel = 0

      # highest prob possible when doing the Bayesian Calcs.
      self.maxOcc = 0.98

      TkMap.__init__(self, cols=cols, rows=rows,
		     frameWidth=width, frameHeight=height,
		     widthMM=widthMM, heightMM=heightMM, title=title)

   def bayesianCertainty (self, distMM, angle, distHitMM,
                          arc, rangeSonarMM, resolutionMM):

   # Returns the probability of being occupied and empty using a "model"
   # of certainty based upon the topology of the sensor, as per that
   # given in Introduction to AI robotics By Robin Murphy

      # basically the further away from the sensor and the center of the
      # beam the less certain the reading will be.

      certaintyFactor = (((float(rangeSonarMM-distMM)/rangeSonarMM) +
                          (float(arc - angle)/arc))/2)

      # We are beyond the useful range of the sensor, we learn nothing
      # from this reading.
      if (distMM >= rangeSonarMM):
         probOcc = .5

      # We have a valid reading, determine the region of the reading.
      # Are we are in the hit region of the sonar?
      elif ((distMM > (distHitMM - resolutionMM)) and
	    (distMM < (distHitMM + resolutionMM))):
	 # Note, unlike Murphy we are only keeping one probability so we
	 # don't want to go below 0.5
	 probOcc = max(certaintyFactor * self.maxOcc,0.5)

      # if not we must be in the empty region of the sonar
      else: # (distMM < (distHitMM - resolutionMM)):
	 # Note, unlike Murphy we are only keeping one probability so we
	 # don't want to go above 0.5
	 probOcc = 1 - max(certaintyFactor,0.5)

      return probOcc

   def sensorHits(self, robot, sensorName):
      """
      This is the main update function, the assumption is that it will
      be called once for each step.

      Point (0,0) is located at the center of the robot.
      Point (offx, offy) is the location of the sensor on the robot.
      Theta is angle of the sensor hit relative to heading 0.
      Dist is the distance of the hit from the sensor.
      Given these values, need to calculate the location of the hit
      relative to the center of the robot (hitx, hity).  

                    .(hitx, hity)
                   /
                  / 
                 /  
           dist /   
               /    
              /     
             /theta 
            .-------
           (offx, offy)
        
      .-->heading 0
      (0,0)
      
      """
      originalUnits = robot.__dict__[sensorName].units
      robot.__dict__[sensorName].units = 'METERS'

      rangeSonarM = robot.__dict__[sensorName].getMaxvalue()

      # -----------------------------------
      for i in range(robot.__dict__[sensorName].count):
         # in MM:
         offx, offy, z, theta, arc = robot.__dict__[sensorName][i].geometry
         # in METERS, because we set it so above:
         dist = robot.__dict__[sensorName][i].value

         # convert to MMs:
         hitx = math.cos(theta) * dist * 1000 + offx
         hity = math.sin(theta) * dist * 1000 + offy

         #  Pyrobot simulation of pioneer:  
	 #   arc 0.087266 (5 deg, simulators/pysim.py, line 2335)
	 #   maxRange = 8m
	 #
         #  Erratic robot: supposedly 18 deg (36 deg total arc)
	 #   arc 0.130900 (7.5 deg, robot/player.py, line 230)
	 #   maxRange = 8m (robot/player.py, line 223)
         # 
	 # NOTE: due to how low these numbers are we assume that arc is
	 # the angle from the center of the beam to the edge, not the
	 # angle from one edge to the other.
	 #
         #  a baseline: 18 deg * (math.pi/180) = .314 rad.
	 #
         # Some reasonable assumptions 
         #  6450mm range, 25.4mm resolution(1 inch)

         self.computeOccupancy(offx, offy, hitx, hity, 
	                       # arc = arc,
	                       arc = .314, 
			       rangeSonarMM = (rangeSonarM * 1000),
	                       resolutionMM = 25.4, 
			       label=i)
      robot.__dict__[sensorName].units = originalUnits


   def computeOccupancy(self, origx, origy, hitx, hity, arc, rangeSonarMM,
                        resolutionMM, label = None):
      """
      Initially only compute occupancies on the line from the robot to
      the sensor hit, i.e., ignore arc size.  

      """

      if self.debug: print "occupancyGrid:", (origx, origy, hitx, hity)

      rise = hity - origy
      if abs(rise) < 0.1:
         rise = 0
      run  = hitx - origx
      if abs(run) < 0.1:
         run = 0
      steps = int(math.ceil(max(abs(rise/self.colScaleMM), \
                            abs(run/self.rowScaleMM))))

      if steps == 0:
         # set the origin of the sensor we are using to be occuped
         self.setGridLocation(row=hitx, col=hity, value=1.0, label=label,
                              absolute=0)
         return

      # set the origin of the sensor we are using to be empty
      self.setGridLocation(row=origx,col=origy,value=0.0,absolute=0)

      stepx = run / float(steps)
      if abs(stepx) > self.rowScaleMM:
         stepx = self.rowScaleMM
         if run < 0:
            stepx *= -1
      stepy = rise / float(steps)
      if abs(stepy) > self.colScaleMM:
         stepy = self.colScaleMM
         if rise < 0:
            stepy *= -1
      currx = origx
      curry = origy
      
      distToHit = math.sqrt(math.pow(rise,2)+math.pow(run,2))
      
      for step in range(steps):
         curry += stepy
         currx += stepx

         distToCurr = math.sqrt(math.pow(currx-origx,2) + 
	                        math.pow(curry-origy,2))

	 # we calculate the radius of the beam at the current point
	 radiusAtCurr = math.tan(arc)*distToCurr

         currentOffset = 0
	 currentOffsetX = 0
	 currentOffsetY = 0

	 # we loop, gradually spreading along the perpendicular to the
	 # path of the beam (the perpendicular which crosses at the
	 # current point) until we reach the width of the beam
	 while(currentOffset < radiusAtCurr):

	    if (currentOffset == 0):
	      angle = 0
	    else:
	      angle = math.atan(currentOffset/distToCurr);

            newValue = self.bayesianCertainty(distMM=distToCurr,
   	                                      angle=angle,
   					      distHitMM=distToHit,
					      arc=arc,
					      rangeSonarMM=rangeSonarMM,
					      resolutionMM=resolutionMM)

	    # we need to set both to the right and to the left of the
	    # beam
            self.setGridLocation(row=(currx+currentOffsetX), 
	                         col=(curry+currentOffsetY), 
				 value=newValue, 
                                 label=label, absolute=0)
            self.setGridLocation(row=(currx-currentOffsetX), 
	                         col=(curry-currentOffsetY), 
				 value=newValue, 
                                 label=label, absolute=0)

	    # note the slope of a perpendicular line is the negative
	    # inverse, thus the swapping of x and y and the negation,
	    # which we negate doesn't matter as we walk both up and down
	    # the perpendicular.
	    currentOffsetX -= stepy
	    currentOffsetY += stepx
	    currentOffset = math.sqrt(math.pow(currentOffsetX,2) +
	                              math.pow(currentOffsetY,2))

   def color(self, value, maxvalue):
      value = 1.0 - value / maxvalue
      color = "gray%d" % int(value * 100.0) 
      return color

   def redraw(self, drawRobot = False, drawLabels = True):
   # The origin of the tkinter coordinate system is at the top left with
   # x increasing along the top edge and y increasing along the left
   # edge:
   #
   #   0,0 ---- +x
   #    |
   #    |
   #   +y
   #
   # This means that x corresponds to the column number and y to the row
   # number.
      maxval = 1
      for row in range(self.rows):
         for col in range(self.cols):
            x = self.cols - 1 - col
            y = row
            
            self.canvas.create_rectangle(int(x * self.colScale),
                                         int(y * self.rowScale),
                                         int((x + 1) * self.colScale),
                                         int((y + 1) * self.rowScale),
                                         width = 0,
                                         fill=self.color(self.getGridLocation(row=row,col=col,absolute=1),
                                                         maxval),
                                         tag = "cell%d" % self.newCellLabel)
            if drawLabels and self.label[row][col]:
               self.canvas.create_text(int((x + .5) * self.colScale),
                                       int((y + .5) * self.rowScale),
                                       text = self.getLabel(row=row,col=col),
                                       fill="yellow",
                                       tag = "cell%d" % self.newCellLabel)
      if drawRobot:
         self.canvas.create_oval( self.width / 2.0 - 10,
                                  self.height / 2.0 - 10,
                                  self.width / 2.0 + 10,
                                  self.height / 2.0 + 10,
                                  fill = "red",
                                  tag = "cell%d" % self.newCellLabel)
         self.canvas.create_rectangle( self.width / 2.0 - 5,
                                       self.height / 2.0 - 5,
                                       self.width / 2.0 + 5,
                                       self.height / 2.0 - 15,
                                       fill = "blue",
                                       tag = "cell%d" % self.newCellLabel)
      self.newCellLabel = not self.newCellLabel
      self.canvas.delete("cell%d" % self.newCellLabel)
      self.update_idletasks()

if __name__ == '__main__':
   lps = LPS(10, 10)
   lps.redraw()
   lps.application = 1
   lps.mainloop()
