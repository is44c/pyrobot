from fake import Fake # cameraDevice
from pyro.camera import Camera, CBuffer # base class
import pyro.system as system
import re, time, os

class FakeCamera(Camera):
   """
   A fake camera.  This will emulate a camera, but instead of
   accessing the hardware, it will load a series of images from file.
   """
   def __init__(self, pattern = None,
                start = 0, stop = 19, char = "?",
                interval = 1.0, visionSystem = None, verbose = 0):
      """
      pattern is a filename with indicators on where to put digits for the
      sequence.  Absolute or relative filenames can be used.

      For example, 'image???-.ppm' would start at 'image000.ppm'
      and continue up to stop.
      
      char is the character that should be replaced in the pattern.

      interval = how often do I get new image

      As an example, to load som-0.ppm through som-19.ppm, we could call
      FakeCamera('vision/snaps/som-?.ppm', 0, 19)
      """
      if pattern == None:
         pattern = "vision/snaps/som-?.ppm"
      self.pattern = pattern
      self.stop = stop
      self.start = start
      self.current = start
      self.setUpdateInterval(interval)
      self.verbose = verbose
      self.lastUpdate = 0
      #create a matchdata object that we will store
      self.match = re.search(re.escape(char) + "+", pattern)
      #create a format string that we can use to replace the
      #replace characters
      if self.match:
         self.fstring = "%%0%dd" % len(self.match.group())
         currname = self.pattern[:self.match.start()] + \
                    self.fstring % self.current + \
                    self.pattern[self.match.end():]
      else:
         currname = self.pattern
      if system.file_exists(currname):
         self.path = ''
      elif system.file_exists( os.getenv('PYRO') + "/" + currname):
         self.path = os.getenv('PYRO') + "/"
      else:
         raise ValueError, "file not found: '%s'" % currname
      if self.verbose:
         print "info: readings file '%s'..." % (self.path + currname)
      self.cameraDevice = Fake(self.path + currname)
      # connect vision system: --------------------------
      self.vision = visionSystem
      self.vision.registerCameraDevice(self.cameraDevice)
      self.width = self.vision.getWidth()
      self.height = self.vision.getHeight()
      self.depth = self.vision.getDepth()
      self.cbuf = self.vision.getMMap()
      # -------------------------------------------------
      self.rgb = (0, 1, 2) # offsets to RGB
      self.format = "RGB"
      Camera.__init__(self, self.width, self.height, self.depth,
                      "Fake Camera View")
      self.devData["subtype"] = "simulated"
      self.devData["source"] = self.pattern
      self.data = CBuffer(self.cbuf)
      self.oldStart = None
      self.oldStop = None

   def pauseButton(self):
      self.freezeFrame()

   def playButton(self):
      self.unFreezeFrame()

   def setUpdateInterval(self, val):
      Camera.setUpdateInterval(self, val)
      self.interval = val

   def freezeFrame(self):
      if self.oldStart == None:
         self.oldStart = self.start
         self.oldStop = self.stop
         self.stop = self.current
         self.start = max(self.current - 1, 0)

   def unFreezeFrame(self):
      if self.oldStart != None:
         self.stop = self.oldStop
         self.start = self.oldStart
         self.oldStart = None
         self.oldStop = None
         
   def _update(self):
      if (self.current < self.stop):
         currentTime = time.time()
         if currentTime - self.lastUpdate > self.interval:
            if self.match:
               currname = self.pattern[:self.match.start()] + \
                          self.fstring % self.current + \
                          self.pattern[self.match.end():]
            else:
               currname = self.pattern
            if self.verbose:
               print "info: readings file '%s'..." % (self.path + currname)
            self.cameraDevice.updateMMap(self.path + currname)
            self.processAll()
            self.current += 1
            self.lastUpdate = currentTime
      else:
         self.current = self.start 

