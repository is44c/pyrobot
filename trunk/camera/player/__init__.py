from pyro.camera import Camera, CBuffer
from pyro.camera.player.playercam import PlayerCam
import threading
import time

class CameraThread(threading.Thread):
    """
    A camera thread class, because feeds it to us
    as fast as we can eat em!
    """
    def __init__(self, runable):
        """
        Constructor, setting initial variables
        """
        self.runable = runable
        self._stopevent = threading.Event()
        self._sleepperiod = 0.001
        threading.Thread.__init__(self, name="CameraThread")
        
    def run(self):
        """
        overload of threading.thread.run()
        main control loop
        """
        while not self._stopevent.isSet():
            self.runable.cameraDevice.updateMMap(0) # 0 = read and throw away; 1 = process
            self._stopevent.wait(self._sleepperiod)

    def join(self,timeout=None):
        """
        Stop the thread
        """
        self._stopevent.set()
        threading.Thread.join(self, timeout)

class PlayerCamera(Camera):
   """
   """
   def __init__(self, host, port, visionSystem = None):
      """
      """
      self.thread = None
      time.sleep(1)
      self.cameraDevice = PlayerCam( host, port)
      # connect vision system: --------------------------
      self.vision = visionSystem
      self.vision.registerCameraDevice(self.cameraDevice)
      self.width = self.vision.getWidth()
      self.height = self.vision.getHeight()
      self.depth = self.vision.getDepth()
      self.cbuf = self.vision.getMMap()
      self.data = CBuffer(self.cbuf)
      self.rgb = (0, 1, 2) # offsets to RGB
      self.format = "RGB"
      Camera.__init__(self, self.width, self.height, self.depth,
                      "Player Camera View")
      self.devData["subtype"] = "player"
      self.data = CBuffer(self.cbuf)
      self.thread = CameraThread(self)
      self.thread.start()

   def _update(self):
       if self.thread:
           self.cameraDevice.updateMMap(1) # read and map
           self.processAll() # need to process filters

   def destroy(self):
      self.thread.join()
