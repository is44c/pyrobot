# Defines PlayerRobot, a subclass of robot

from pyro.robot import *
from math import pi, cos, sin
from os import getuid
from pyro.robot.driver.player import *

PIOVER180 = pi / 180.0
DEG90RADS = 0.5 * pi
COSDEG90RADS = cos(DEG90RADS) / 1000.0
SINDEG90RADS = sin(DEG90RADS) / 1000.0

class PlayerRobot(Robot):
    def __init__(self, name = "Player", port = 6665):
        Robot.__init__(self, name, "player") # robot constructor
        self.simulated = 1
        self.port = port
        self.inform("Loading Player robot interface...")
        self.connect()
        self.sensorSet = {'all': range(16),
                          'front': (3, 4),
                          'front-left' : (1,2,3),
                          'front-right' : (4, 5, 6),
                          'front-all' : (1,2, 3, 4, 5, 6),
                          'left' : (0, 15), 
                          'right' : (7, 8), 
                          'left-front' : (0,), 
                          'right-front' : (7, ),
                          'left-back' : (15, ),
                          'right-back' : (8, ),
                          'back-right' : (9, 10, 11),
                          'back-left' : (12, 13, 14), 
                          'back' : (11, 12),
                          'back-all' : ( 9, 10, 11, 12, 13, 14)}
        self.z = 0.0
        self.y = 0.0
        self.x = 0.0
        self.senses = {}
        simulated = self.simulated
	# robot senses (all are functions):
        self.senses['robot'] = {}
        self.senses['robot']['simulator'] = lambda dev, x = simulated: x
        self.senses['robot']['stall'] = lambda dev: self.stall
        self.senses['robot']['x'] = self.getX
        self.senses['robot']['y'] = self.getY
        self.senses['robot']['z'] = self.getZ
        self.senses['robot']['th'] = self.getTh # in degrees
        self.senses['robot']['thr'] = self.getThr # in radians
        self.senses['robot']['type'] = lambda dev: 'Player'
        self.senses['robot']['units'] = lambda dev: 'METERS'
        self.senses['robot']['name'] = lambda dev, x = name: name
        
        self.senses['sonar'] = {}
        self.sonarGeometry = self.dev.get_sonar_geometry()
        self.sonarAngles = map(lambda lst: lst[2], self.sonarGeometry)
        self.senses['sonar']['count'] = lambda dev: len(self.sonarGeometry)
        self.senses['sonar']['type'] = lambda dev: 'range'
        
        # location of sensors' hits:
        self.senses['sonar']['x'] = lambda dev, pos: self.localXDev(dev, pos)
        self.senses['sonar']['y'] = lambda dev, pos: self.localYDev(dev, pos)
        self.senses['sonar']['z'] = lambda dev, pos: 0.03 # meters
        self.senses['sonar']['value'] = lambda dev, pos: self.rawToUnits(dev, self.dev.sonar[0][pos], 'sonar')
        self.senses['sonar']['maxvalue'] = lambda dev: self.rawToUnits(dev, 2990, 'sonar')
        self.senses['sonar']['units'] = lambda dev: "ROBOTS"
        
        # location of origin of sensors:
        self.senses['sonar']['ox'] = lambda dev, pos: self.rawToUnits(dev, self.sonarGeometry[pos][0], 'sonar')
        self.senses['sonar']['oy'] = lambda dev, pos: self.rawToUnits(dev, self.sonarGeometry[pos][1], 'sonar')
        self.senses['sonar']['oz'] = lambda dev, pos: 0.03 # meters
        self.senses['sonar']['th'] = lambda dev, pos: self.sonarGeometry[pos][2] * PIOVER180 
        #         # in radians:
        self.senses['sonar']['arc'] = lambda dev, pos, \
                                      x = (7.5 * PIOVER180) : x
        
        #         if self.params.haveFrontBumpers() or self.params.haveRearBumpers():
        #             # bumper sensors
        #             self.senses['bumper'] = {}
        #             self.senses['bumper']['type'] = lambda dev: 'tactile'
        #             self.senses['bumper']['count'] = lambda : self.params.numFrontBumpers() + self.params.numRearBumpers()
        #             self.senses['bumper']['x'] = lambda dev, pos: 0
        #             self.senses['bumper']['y'] = lambda dev, pos: 0
        #             self.senses['bumper']['z'] = lambda dev, pos: 0
        #             self.senses['bumper']['th'] = lambda dev, pos: 0 
        #             self.senses['bumper']['value'] = lambda dev, pos: 0
        
        #         self.controls['gripper'] = ArGripper(self.dev)
        #         # gripper sensors
        # 	self.senses['gripper'] = {}
        #         self.senses['gripper']['type'] = lambda dev: 'special'
        
        # Make a copy, for default:
        self.senses['range'] = self.senses['sonar']
        self.senses['self'] = self.senses['robot']

        self.controls['move'] = self.moveDev
        self.controls['translate'] = self.translateDev
        self.controls['rotate'] = self.rotateDev
        self.controls['update'] = self.updateDev
        #self.controls['localize'] = self.localizeDev
        console.log(console.INFO,'Player control drivers loaded')
        
    def translate(self, translate_velocity):
        self.translateDev(self.dev, translate_velocity)

    def translateDev(self, dev, translate_velocity):
        dev.set_speed(translate_velocity * 1100.0, None, None)

    def rotate(self, rotate_velocity):
        self.rotateDev(self.dev, rotate_velocity)

    def rotateDev(self, dev, rotate_velocity):
        dev.set_speed(None, None, rotate_velocity * 75.0)

    def move(self, translate_velocity, rotate_velocity):
        self.moveDev(self.dev, translate_velocity, rotate_velocity)

    def moveDev(self, dev, translate_velocity, rotate_velocity):
        dev.set_speed(translate_velocity * 1100.0,
                      0,
                      rotate_velocity * 75.0)
        
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

    def getX(self, dev):
        return self.x

    def getY(self, dev):
        return self.y
    
    def getZ(self, dev):
        return self.z
    
    def getTh(self, dev):
        return self.th
    
    def getThr(self, dev):
        return self.thr

    def update(self):
        self.updateDev(self.dev)
    
    def updateDev(self, dev):
        data = dev.get_position()
        pos, speeds, stall = data
        # (xpos, ypos, th), (xspeed, yspeed, rotatespeed), stall
        self.x = pos[0] / 1000.0
        self.y = pos[1] / 1000.0
        self.th = pos[2] # degrees
        self.thr = self.th * PIOVER180
        self.stall = stall
        
    def _draw(self, options, renderer): # overloaded from robot
        #self.setLocation(self.senses['robot']['x'], \
        #                 self.senses['robot']['y'], \
        #                 self.senses['robot']['z'], \
        #                 self.senses['robot']['thr'] )
        renderer.xformPush()
        renderer.color((1, 0, 0))
        #print "position: (", self.get('robot', 'x'), ",",  \
        #      self.get('robot', 'y'), ")"
        
        #renderer.xformXlate((self.get('robot', 'x'), \
        #                     self.get('robot','y'), \
        #                     self.get('robot','z')))
        renderer.xformRotate(self.get('robot', 'th'), (0, 0, 1))
        
        renderer.xformXlate(( 0, 0, .15))
        
        renderer.box((-.25, .25, 0), \
                     (.25, .25, 0), \
                     (.25, -.25, 0), \
                     (-.25, .25, .35)) # d is over a, CW
        
        renderer.color((1, 1, 0))
        
        renderer.box((.13, -.05, .35), \
                     (.13, .05, .35), \
                     (.25, .05, .35), \
                     (.13, -.05, .45)) # d is over a, CW
        
        renderer.color((.5, .5, .5))
        
        # wheel 1
        renderer.xformPush()
        renderer.xformXlate((.25, .3, 0))
        renderer.xformRotate(90, (1, 0, 0))
        renderer.torus(.06, .12, 12, 24)
        renderer.xformPop()
        
        # wheel 2
        renderer.xformPush()
        renderer.xformXlate((-.25, .3, 0))
        renderer.xformRotate(90, (1, 0, 0))
        renderer.torus(.06, .12, 12, 24)
        renderer.xformPop()
        
        # wheel 3
        renderer.xformPush()
        renderer.xformXlate((.25, -.3, 0))
        renderer.xformRotate(90, (1, 0, 0))
        renderer.torus(.06, .12, 12, 24)
        renderer.xformPop()
        
        # wheel 4
        renderer.xformPush()
        renderer.xformXlate((-.25, -.3, 0))
        renderer.xformRotate(90, (1, 0, 0))
        renderer.torus(.06, .12, 12, 24)
        renderer.xformPop()        
        
        # sonar
        renderer.xformPush()
        renderer.color((0, 0, .7))
        for i in range(self.get('sonar', 'count')):
            y1, x1, z1 = -self.get('sonar', 'x', i), \
                         -self.get('sonar', 'y', i), \
                         self.get('sonar', 'z', i)
            #y2, x2, z2 = -self.get('sonar', 'ox', i), \
            #             -self.get('sonar', 'oy', i), \
            #             self.get('sonar', 'oz', i)
            # FIXME: what are the actual positions of sensor x,y?
            x2, y2, z2 = 0, 0, z1
            arc    = self.get('sonar', 'arc', i) # in radians
            renderer.ray((x1, y1, z1), (x2, y2, z2), arc)
            
        renderer.xformPop()        
        
        # end of robot
        renderer.xformPop()

    def getOptions(self): # overload 
        pass

    def connect(self):
        self.dev = player('localhost', port=self.port)
        print "Device: ", self.dev
        self.dev.start('position')
        self.dev.start('sonar')
        #self.localize(0.0, 0.0, 0.0)
        
        #     def localize(self, x = 0.0, y = 0.0, th = 0.0):
        #         self.diffX = x - self.dev.getX()
        #         self.diffY = y - self.dev.getY()
        #         self.diffTh = th - self.dev.getTh()
        
    def disconnect(self):
        print "Disconnecting..."

    def rawToUnits(self, dev, raw, name):
        raw = raw / 1000.0
        if name == 'sonar':
            val = min(max(raw, 0.0), 2.99)
        else:
            raise 'InvalidType', "Type is invalid"
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
            raise 'InvalidType', "Units are set to invalid type"

    
if __name__ == '__main__':
    myrobot = PlayerRobot()
    myrobot.update()
    #myrobot.translate(.2)
    #myrobot.translate(.0)
    #myrobot.disconnect()
