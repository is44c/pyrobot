from pyrobot.camera.fake import FakeCamera
from pyrobot.vision.cvision import VisionSystem

def INIT(robot):
    return {"camera": FakeCamera(pattern = "vision/blimp/bullseye-?.pbm",
                                 start = 1,
                                 stop = 5,
                                 interval = 1,
                                 visionSystem = VisionSystem())}
