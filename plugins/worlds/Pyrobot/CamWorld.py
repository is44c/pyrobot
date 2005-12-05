"""
A PyrobotSimulator world. A large room with two robots and
two lights.

(c) 2005, PyroRobotics.org. Licensed under the GNU GPL.
"""

from pyrobot.simulators.pysim import *

def INIT():
    # (width, height), (offset x, offset y), scale:
    sim = TkSimulator((441,434),(22,420), 40.357554)  
    # x1, y1, x2, y2 in meters:
    #sim.addBox(0, 0, 10, 10)
    # port, name, x, y, th, bounding Xs, bounding Ys, color
    # (optional TK color name):
    sim.addRobot(60000, TkPioneer("Pioneer1",
                                  1, 1, 5.42,
                                  ((.225, .225, -.225, -.225),
                                   (.175, -.175, -.175, .175)),
                                  "red"))
    # add some sensors:
    # x, y relative to body center (beyond bounding box):
    sim.robots[0].addDevice(PioneerFrontSonars())
    sim.robots[0].addDevice(Camera(60, 40, 60 * math.pi/180, -60 * math.pi/180, 0, 0, 0))

    sim.addRobot(None, TkPuck("Puck1", 5, 5, 0, ((.05, .05, -.05, -.05), (.05, -.05, -.05, .05)), "purple"))
    sim.addRobot(None, TkPuck("Puck2", 6, 6, 0, ((.05, .05, -.05, -.05), (.05, -.05, -.05, .05)), "blue"))
        
    return sim
