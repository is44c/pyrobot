# Defines PlayerRobot, a subclass of robot

from pyro.robot import *
from math import pi, cos, sin
from os import getuid
from pyro.robot.driver.player import *
import random, time

PIOVER180 = pi / 180.0
DEG90RADS = 0.5 * pi
COSDEG90RADS = cos(DEG90RADS) / 1000.0
SINDEG90RADS = sin(DEG90RADS) / 1000.0

from pyro.robot.service import Service, ServiceError

class PlayerService(Service):
    def __init__(self, dev, name):
        Service.__init__(self)
        self.dev = dev
        self.name = name

    def startService(self):
        try:
            self.dev.start(self.name)
        except:
            print "Device not supported: '%s'" % self.name
            self.dev = 0

    def checkService(self):
        if self.dev == 0:
            raise ServiceError, "Service '%s' not available" % self.name

    def stopService(self):
        self.checkService()
        self.dev.stop(self.name)
        self.dev.__dict__[self.name] = {}

    def getServiceData(self, pos = 0):
        self.checkService()
        return self.dev.__dict__[self.name][pos]

    def getServiceState(self):
        self.checkService()
        if self.dev.__dict__[self.name] != {}:
            return "started"
        else:
            return "stopped"

    def getPose(self):
        function = self.dev.__dict__[ "get_%s_pose" ]
        if function != None:
            function(self.dev)
        else:
            raise ServiceError, "Function 'getPose' is not available for service '%s'" % self.name


    def setPose(self, xMM, yMM, thDeg):
        function = self.dev.__dict__[ "set_%s_pose" ]
        if function != None:
            function( self.dev, xMM, yMM, thDeg)
        else:
            raise ServiceError, "Function 'setPose' is not available for service '%s'" % self.name

class PlayerCommService(PlayerService):
    def sendMessage(self, message):
        if self.dev.comms == {}:
            print "Need to startService('comm') in robot: message not sent"
            return
        self.dev.send_message(message)

    def getMessages(self):
        if not 'comms' in dir(self.dev) or self.dev.comms == {}:
            raise ServiceError, "Need to startService('comm') in robot"
        #if self.dev.comms[0] != '':
        #    self.update() # this is update in robot
        tmp = self.messages
        # reset queue:
        self.messages = []
        return tmp

class PlayerGripperService(PlayerService):
    # Gripper functions
    #these also exist: 'gripper_carry', 'gripper_press', 'gripper_stay',

    def open(self):
        return self.dev.gripper_open() 

    def close(self):
        return self.dev.gripper_close() 

    def stopMoving(self):
        return self.dev.gripper_stop()

    def liftUp(self):
        return self.dev.gripper_up()

    def liftDown(self):
        return self.dev.gripper_down()

    def liftStop(self):
        return self.dev.gripper_stop()

    def store(self):
        return self.dev.gripper_store() 

    def deploy(self):
        return self.dev.gripper_deploy()

    def halt(self):
        return self.dev.gripper_halt()

    def getState(self):
        return self.dev.is_paddles_closed() # help!

    def getBreakBeamState(self):
        sum = 0
        sum += self.dev.is_ibeam_obstructed() * 2 #FIX: which?
        sum += self.dev.is_obeam_obstructed()
        return sum

    def isClosed(self): # FIX: add this to aria
        return self.dev.is_paddles_closed() #ok

    def isMoving(self):
        return self.dev.is_paddles_moving() #ok

    def isLiftMoving(self):
        return self.dev.is_lift_moving() # ok

    def isLiftMaxed(self):
        return self.dev.is_lift_up() # ok

class PlayerBase(Robot):
    def __init__(self, name = "Player", port = 6665):
        Robot.__init__(self, name, "player") # robot constructor
        self.simulated = 1
        self.pause = 0.0
        self.port = port
        self.inform("Loading Player robot interface...")
        self.name = name
        self.connect() # set self.dev to player robot
        # default values
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.thr = 0.0
        self.stall = 0
        self.messages = []
        self.noise = .05 # 5 % noise
        self.supports["blob"] = PlayerService(self.dev, "blobfinder")
        self.supports["comm"] = PlayerService(self.dev, "comms")
        self.supports["gripper"] = PlayerGripperService(self.dev, "gripper")
        self.supports["power"] = PlayerService(self.dev, "power")
        self.supports["position"] = PlayerService(self.dev, "position")
        self.supports["sonar"] = PlayerService(self.dev, "sonar")
        self.supports["laser"] = PlayerService(self.dev, "laser")
        self.supports["ptz"] = PlayerService(self.dev, "ptz")
        self.supports["gps"] = PlayerService(self.dev, "gps")
        self.supports["bumper"] = PlayerService(self.dev, "bumper")
        self.supports["truth"] = PlayerService(self.dev, "truth")
        
    def translate(self, translate_velocity):
        self.translateDev(self.dev, translate_velocity)

    def translateDev(self, dev, translate_velocity):
        self.update()
        dev.set_speed(translate_velocity * 900.0, None, None)
        time.sleep(self.pause)

    def rotate(self, rotate_velocity):
        self.rotateDev(self.dev, rotate_velocity)

    def rotateDev(self, dev, rotate_velocity):
        self.update()
        dev.set_speed(None, None, rotate_velocity * 65.0)
        time.sleep(self.pause)

    def move(self, translate_velocity, rotate_velocity):
        self.moveDev(self.dev, translate_velocity, rotate_velocity)

    def moveDev(self, dev, translate_velocity, rotate_velocity):
        self.update()
        dev.set_speed(translate_velocity * 900.0,
                      0,
                      rotate_velocity * 65.0)
        time.sleep(self.pause)

    # FIX: either sonar values are changing between calls to X, Y
    # or sin/cos values are not taking into account offset from center
        
    def localXDev(self, dev, pos):
        thr = (self.sonarGeometry[pos][2] + 90.0) * PIOVER180
        dist = self.rawToUnits(dev, self.dev.sonar[0][pos], 'sonar')
        x = self.rawToUnits(dev, self.sonarGeometry[pos][0], 'sonar')
        return cos(thr) * dist

    def localYDev(self, dev, pos):
        thr = (self.sonarGeometry[pos][2] - 90.0) * PIOVER180
        dist = self.rawToUnits(dev, self.dev.sonar[0][pos], 'sonar')
        y = self.rawToUnits(dev, self.sonarGeometry[pos][1], 'sonar') 
        return sin(thr) * dist

    def getX(self, dev = 0):
        return self.x

    def getY(self, dev = 0):
        return self.y
    
    def getZ(self, dev = 0):
        return self.z
    
    def getTh(self, dev = 0):
        return self.th
    
    def getThr(self, dev = 0):
        return self.thr

    def update(self):
        self.updateDev(self.dev)
    
    def updateDev(self, dev):
        self._update()
        data = dev.get_position()
        pos, speeds, stall = data
        # (xpos, ypos, th), (xspeed, yspeed, rotatespeed), stall
        self.x = pos[0] / 1000.0
        self.y = pos[1] / 1000.0
        self.th = pos[2] # degrees
        self.thr = self.th * PIOVER180
        self.stall = stall
        try:
            if self.dev.comms[0] != '':
                for i in self.dev.comms:
                    self.messages.append(self.dev.get_comms())
        except:
            # comms interface not on
            pass
        
    def getOptions(self): # overload 
        pass

    def disconnect(self):
        print "Disconnecting..."

    def rawToUnits(self, dev, raw, name, noise):
        raw = raw / 1000.0
        if noise > 0:
            if random.random() > .5:
                raw += (raw * (noise * random.random()))
            else:
                raw -= (raw * (noise * random.random()))
        if name == 'sonar':
            val = min(max(raw, 0.0), 2.99)
        else:
            raise TypeError, "Type is invalid"
        if self.senses[name]['units'](dev) == "ROBOTS":
            return val / 0.75 # Pioneer is about .5 meters diameter
        elif self.senses[name]['units'](dev) == "MM":
            return val * 1000.0
        elif self.senses[name]['units'](dev) == "CM":
            return (val) * 100.0 # cm
        elif self.senses[name]['units'](dev) == "METERS" or \
                 self.senses[name]['units'](dev) == "RAW":
            return (val) 
        elif self.senses[name]['units'](dev) == "SCALED":
            return val / 2.99
        else:
            raise TypeError, "Units are set to invalid type"

    def localize(self, x = 0, y = 0, th = 0):
        """
        Set robot's internal pose to x (meters), y (meters),
        th (radians)
        """
        self.dev.set_odometry(x * 1000, y * 1000, th)
        self.x = x
        self.y = y
        self.th = th
        self.thr = self.th * PIOVER180

    # Gripper functions
    #these also exist: 'gripper_carry', 'gripper_press', 'gripper_stay',

    
if __name__ == '__main__':
    myrobot = PlayerBase()
    myrobot.update()
    #myrobot.translate(.2)
    #myrobot.translate(.0)
    #myrobot.disconnect()
