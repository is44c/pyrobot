# A Behavior-based system

from pyro.brain.fuzzy import *
from pyro.brain.behaviors import *
from pyro.brain.behaviors.core import *  # Stop

import math
from random import random
import time

class Avoid (Behavior):
    def init(self): # called when created
        self.lasttime = time.mktime(time.localtime())
        self.count = 0

    def direction(self, dir, dist):
        if dir < 0.0:
            return 1.0 - dir
        else:
            return -1.0 - dir

    def update(self):
        print self.getEngine().config.get("experiment", "size")
        if self.count == 50:
            currtime = time.mktime(time.localtime())
            print "=======  50 Loops. Average time per loop =", (currtime - self.lasttime)/50.0, "seconds."
            self.count = 0
            self.lasttime =  time.mktime(time.localtime())
        else:
            self.count += 1
        # Normally :
        close_dist, close_angl = self.getRobot().get('range', 'value', 'front-all', 'close')
        close_angl /= (math.pi)
        #print "Closest distance =", close_dist, "angle =", close_angl
        max_sensitive = self.getRobot().get('range', 'maxvalue') * 0.8
        self.IF(Fuzzy(0.0, max_sensitive) << close_dist, 'translate', 0.0, "TooClose")
        self.IF(Fuzzy(0.0, max_sensitive) >> close_dist, 'translate', .2, "Ok")
        self.IF(Fuzzy(0.0, max_sensitive) << close_dist, 'rotate', self.direction(close_angl, close_dist), "TooClose")
        self.IF(Fuzzy(0.0, max_sensitive) >> close_dist, 'rotate', 0.0, "Ok")

class state1 (State):
    def init(self):
        #self.add(StraightBehavior(1))
        self.add(Avoid(1, {'translate': .3, 'rotate': .3}))
        #self.Effects('translate', .3) 
        #self.Effects('rotate', .3)

        print "initialized state", self.name

def INIT(engine): # passes in robot, if you need it
    brain = BehaviorBasedBrain({'translate' : engine.robot.translate, \
                                'rotate' : engine.robot.rotate, \
                                'update' : engine.robot.update }, engine)
    # add a few states:
    brain.add(state1()) # non active

    # activate a state:
    brain.activate('state1') # could have made it active in constructor
    brain.init()
    return brain
