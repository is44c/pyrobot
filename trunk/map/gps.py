from pyro.map.tkmap import TkMap
from math import cos, sin, pi, sqrt, tanh

class GPS(TkMap):
   """
   GUI for visualizing the global perceptual space of a robot.
   """

   """
   ---------------------------------
   __init__: initialize a GPS window
   ----------------------------------
     parent: root of GUI
     maxrange: maximum range of range sensors, in MM
     cols: number of columns in map
     rows: number of rows in map
     value:
     widthMM: actual distance represented by a column (in MMs)
     heightMM: actual distance represented by a row (in MMs)
     title: window title
   """
   def __init__(self, parent, maxrange = 2000, cols=400, rows=400, value = 0.5,
                widthMM = 10000, heightMM = 10000,
                title = "Global Perceptual Space"):
      """ Pass in parent GUI, maxrange, grid cols, grid cells, and total cols/rows in MM"""
      self.step = 0
      self.changedValues = ()
      # 2000 is the max dist of a Pioneer sonar (in MMs) 
      self.range = maxrange
      # sonar model is broken into 3 regions - these are the boundaries
      self.reg2max = self.range * 0.3
      self.reg1max = self.range * 0.75
      self.field = 15
      # max probability of occupancy
      self.maxOccupied = 0.98
      # create the map
      TkMap.__init__(self, parent, cols, rows, value,
                     cols, rows,
                     widthMM, heightMM, title)


   def probOccupied( self, dist, angle ):
      """
      -------------------------------------------------------------------------
      probOccupied: compute the probability that a given location is occupied
      -------------------------------------------------------------------------
      dist: distance from robot to location
      angle: angle from robot from zero heading of sensor
      """
      # figure out what region we are in
      if( dist <= self.reg2max ):
         region = 2
      elif( dist > self.reg2max and dist <= self.reg1max ):
         region = 1
      else:
         region = 3

      # set range and field based on previous knowledge and current readings
      if( region == 1 ):
         r = (self.range - dist)/self.range
         f = (self.field - angle)/self.field 
         #print "        r:", r, "    f", f
         return( ((r+f) / 2) * self.maxOccupied )

      elif( region == 2 ):
         return( 1.0 - self.probEmpty( dist, angle ) )
      elif( region == 3 ):
         return( -1 )

   def probEmpty( self, dist, angle ):
      """
      -------------------------------------------------------------------------
      probEmpty: compute the probability that a given location is NOT occupied
      -------------------------------------------------------------------------
      dist: distance from robot to location
      angle: angle from robot from zero heading of sensor
      """
      # figure out what region
      if( dist <= self.reg2max ):
         region = 2
      elif( dist > self.reg2max and dist <= self.reg1max ):
         region = 1
      else:
         region = 3

      # calculate and return probability based on Bayes Rule
      if( region == 1 ):
         return( 1.0 - self.probOccupied( dist, angle ) )
      elif( region == 2 ):
         r = (self.range - dist)/self.range
         f = (self.field - angle)/self.field 
         #print "        r:", r, "    f", f
         
         return((r+f) / 2)
      # nothing to be done in region 3 - since we didn't find anything
      elif( region == 3 ):
         return( -1 )

   def getProb( self, dist, angle, prevVal ):
      """
      -------------------------------------------------------------------------
      getProb: compute the probability that a given location is NOT occupied
      -------------------------------------------------------------------------
      dist: distance from robot to location
      angle: angle from robot from zero heading of sensor
      prevVal: get probability from previous iterations
      """
      # get region
      if( dist <= self.reg2max ):
         region = 2
      elif( dist <= self.reg1max ):
         region = 1
      else:
         region = 3

      # inverse probability of old value
      prevValInv = 1 - prevVal
      # get new probabilities
      probOcc = self.probOccupied( dist, angle )
      probEmpty = self.probEmpty( dist, angle )

      # Calculate Bayesian probability
      if( probOcc == -1 or probEmpty == -1 ):
         return( prevVal )
      else:
         return( (probOcc * prevVal) / ( (probOcc * prevVal) + (probEmpty * prevValInv) ) )
 
   def updateFromLPS(self, lps, robot):
      """
      -------------------------------------------------------------------------
      updateFromLPS: update values based on LPS
      -------------------------------------------------------------------------
      lps: class containing sensor data
      robot: contains information about current robot
      """
      # things we will need:
      #grid, x, y, th, cellxMM, cellyMM:
      #self.canvas.delete("old")
      x = robot.getX() * 1000
      y = robot.getY() * 1000
      thr = robot.getThr()
      # In GPS, the origin is at the bottom left corner.
      # This matches the way world files are specified.
      ypos = self.rows - int(y / self.rowScaleMM) - 1
      xpos = int(x / self.colScaleMM)
      ## draw out the sonar readings from the LPS
      for i in range(lps.rows):
         for j in range(lps.cols):
            # y component is negative because y up is positive
            xMM = (j - (lps.cols / 2)) * lps.colScaleMM
            yMM = -1 * (i - (lps.rows / 2)) * lps.rowScaleMM
            # cos(0) = 1, sin(0) = 0
            xrot = (xMM * cos(thr) - yMM * sin(thr))
            yrot = (xMM * sin(thr) + yMM * cos(thr))

            # current location in question
            xhit = x + xrot 
            yhit = y + yrot 

            # current cell in question
            xcell = int(xhit / self.colScaleMM)
            ycell = self.rows - int(yhit / self.rowScaleMM) - 1

            # distance from robot to current location
            dist = sqrt(xrot*xrot + yrot*yrot)
            
            # robot's position
            oldx = int(xcell * self.colScale) + (self.rows/3)
            oldy = int(ycell * self.rowScale) - (self.cols/3)

            # current location value
            oldVal = self.getGridLocation( oldx, oldy, 1 )
            # _NoConvert( oldx, oldy )

            # LPS grid only holds 3 values:
            #  0:updated and nothing there
            #  0.5:not updated - no idea what's here
            #  1: updated and something was hit

            # lps saw something here - update it's probability 
            if lps.grid[i][j] == 1.0:
               occ = lps.grid[i][j] # occ = self.getProb( dist, 0, oldVal )
               self.plotCell( xcell, ycell, occ ) #"black")
               
            # lps never updated - leave these locations along
            elif lps.grid[i][j] == 0.5:
               self.plotCell( xcell, ycell, oldVal ) #
            # lps updated this, and found nothing - deteriorate value
            else:
               self.plotCell( xcell, ycell, oldVal * 0.1 )

      ## now draw out the path of the robot
      ## if self.inRange(ypos, xpos):
      ##    self.plotCell(xpos, ypos, 2.0 ) # "red")

      ## print "--------DONE WITH LPS---------"
 
   def plotCell( self, xpos, ypos, value ):
      """
      -------------------------------------------------------------------------
      plotCell: sets a point up to be plotted - no longer actually plots a cell
      this does, however, update the grid location with the new value
      -------------------------------------------------------------------------
      xpos: x coordinate on map
      ypos: y coordinate on map
      value: new value for [x,y]
      """
      # figure out which cell this represents
      xCell = int(xpos * self.colScale) + (self.rows/3)
      yCell = int(ypos * self.rowScale) - (self.cols/3)

      # get the current value
      oldVal = self.getGridLocation( xCell, yCell, 1 ) # _NoConvert( xCell, yCell )

      # no need to update if value isn't changing
      if( value != oldVal ):
         # don't want to get too small in case something changes
         if( value < 0.001 ):
            value = 0.001
         # don't want to get too large, in case something changes
         elif( value > 0.98 ):
            value = 0.98

         # set the value
         self.setGridLocation( xCell, yCell, value, None, 1 )
         # _NoConvert( xCell, yCell, value, None )

         # update the changedValues tuple with this new coordinate set
         self.changedValues = self.changedValues, (xCell, yCell)

   def redraw( self ):
      """
      -------------------------------------------------------------------------
      redraw: redraws the graphical map
      -------------------------------------------------------------------------
      """
      # traverse through updated points - redraw those points on map
      while( len( self.changedValues ) ):
         self.changedValues, (row,col) = self.changedValues
         val = self.getGridLocation( row, col, 1 ) # _NoConvert( row, col )

##       # draws out robot path -- no longer used!!
##          if( val == 2.0 ):
##             color = "red"
##             self.canvas.create_rectangle(row-1, col-1, row+1, col+1,
##                                          width = 0,
##                                          fill=color, tag = "old")
##          else:
         color = "gray%d" % ((1-val)*100)                           
         self.canvas.create_rectangle(row, col, row+1, col+1,
                                      width = 0,
                                      fill=color, tag = "old")
            
if __name__ == '__main__':
   import Tkinter
   root = Tkinter.Tk()
   gps = GPS(root, 50, 50)
   gps.application = 1
   gps.mainloop()