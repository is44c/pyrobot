"""
A PyrobotSimulator world. A large room with two robots and
two lights.

(c) 2005, PyroRobotics.org. Licensed under the GNU GPL.
"""

from pyrobot.simulators.pysim import TkSimulator, TkPioneer, \
     PioneerFrontSonars, PioneerFrontLightSensors

def INIT():
    # (width, height) pixels, (offset x, offset y) pixels, scale:
    sim = TkSimulator((600,600), (30, 586), 13.7) 
    sim.addBox(0, 0, 40, 40) # x1, y1, x2, y2 in meters
    sim.addLight(5, 5, 1) # (x, y) meters, brightness usually 1 (1 meter radius)
    sim.addLight(5, 30, 1) # (x, y) meters, brightness usually 1 (1 meter radius)
    sim.addWall(0, 20, 10, 10)  # x, y in meters
    # port, name, x, y, th, bounding Xs, bounding Ys, color (optional TK color name):
    sim.addRobot(60000, TkPioneer("RedPioneer",
                                  15, 30, 0.0,
                                  ((.75, .75, -.75, -.75),
                                   (.5, -.5, -.5, .5))))
    # port, name, x, y, th, bounding Xs, bounding Ys, color (optional TK color name):
    sim.addRobot(60001, TkPioneer("BluePioneer",
                                  30, 35, 1.5,
                                  ((.75, .75, -.75, -.75),
                                   (.5, -.5, -.5, .5)),
                                  color="blue"))
    # add some sensors
    sim.robots[0].addDevice(PioneerFrontSonars())
    sim.robots[0].addDevice(PioneerFrontLightSensors())
    sim.robots[1].addDevice(PioneerFrontSonars())
    sim.robots[1].addDevice(PioneerFrontLightSensors())
    return sim # return the simulation object