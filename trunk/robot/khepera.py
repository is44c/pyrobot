# Defines KheperaRobot, a subclass of robot

from pyro.robot import *
from pyro.system.SerialConnection import *
import pyro.robot.driver as driver
import pyro.gui.console as console
import string
import array
import math
import termios
from time import sleep

class KheperaRobot(Robot):
    def __init__(self, name = None, simulator = 0): # 0 makes it real
        Robot.__init__(self, name, "khepera") # robot constructor
        if simulator == 1:
            self.sc = "Fix: Make Simulated Connection"
        else:
            self.sc = SerialConnection("/dev/ttyS1", termios.B38400)
            #self.sc = SerialConnection("/dev/ttyS1", termios.B115200)
            #self.sc = SerialConnection("/dev/ttyS1", termios.B57600)
        self.dev = self # pointer to self
        self.sensorGroups = {'all' : [ (0, 'ir'), (4, 'ir'),
                                       (1, 'ir'), (5, 'ir'),
                                       (2, 'ir'), (6, 'ir'),
                                       (3, 'ir'), (7, 'ir') ],
                             'front' : [(2, 'ir'), (3, 'ir')], 
                             'front-left' : [(0, 'ir'), (1, 'ir')], 
                             'front-right' : [(4, 'ir'), (5, 'ir')],
                             'front-all' : [(1, 'ir'), (2, 'ir'),
                                            (3, 'ir'), (4, 'ir')], 
                             'left' : [(0, 'ir')], 
                             'right' : [(5, 'ir')], 
                             'left-front' : [(0, 'ir')], 
                             'rightfront' : [(5, 'ir')], 
                             'left-back' : [(7, 'ir')], 
                             'right-back' : [(6, 'ir')], 
                             'back-left' : [(7, 'ir')], 
                             'back-right' : [(6, 'ir')], 
                             'back-all' : [(6, 'ir'), (7, 'ir')], 
                             'back' : [(6, 'ir'), (7, 'ir')]} 
        self.senseData = {}
        self.lastTranslate = 0
        self.lastRotate = 0
        # This could go as high as 127, but I am keeping it small
        # to be on the same scale as larger robots. -DSB
        self.translateFactor = 30
        self.rotateFactor = 12

        self.inform("Loading Khepera robot interface...")

	# robot senses (all are functions):
        self.senses['robot'] = {}
        self.senses['robot']['simulator'] = lambda self, x = simulator: x
        self.senses['robot']['stall'] = self.isStall
        self.senses['robot']['x'] = self.getX
        self.senses['robot']['y'] = self.getY
        self.senses['robot']['z'] = self.getZ
        self.senses['robot']['th'] = self.getTh # in degrees
        self.senses['robot']['thr'] = self.getThr # in radians
	self.senses['robot']['type'] = lambda self: 'khepera'

	self.senses['robot']['name'] = lambda self: 'khepera-1'

	self.senses['ir'] = {}
	self.senses['ir']['count'] = lambda self: 8
	self.senses['ir']['type'] = lambda self: 'range'

	# location of sensors' hits:
	self.senses['ir']['x'] = self.getIRXCoord
	self.senses['ir']['y'] = self.getIRYCoord
	self.senses['ir']['z'] = lambda self, pos: 0.25
	self.senses['ir']['value'] = self.getIRRange
        self.senses['ir']['all'] = self.getIRRangeAll
        self.senses['ir']['maxvalue'] = lambda self: 60.0 # in mm
	self.senses['ir']['flag'] = self.getIRFlag
        self.senses['ir']['units'] = lambda self: "ROBOTS"

	# location of origin of sensors:
        self.senses['ir']['ox'] = self.light_ox
	self.senses['ir']['oy'] = self.light_oy
	self.senses['ir']['oz'] = lambda self, pos: 0.25 # meters
	self.senses['ir']['th'] = self.light_th
        # in radians:
        self.senses['ir']['arc'] = lambda self, pos, \
                                      x = (5 * math.pi / 180) : x
        # Make a copy, for default:
        self.senses['range'] = self.senses['ir']

	self.senses['light'] = {}
	self.senses['light']['count'] = lambda self: 8
	self.senses['light']['type'] = lambda self: 'measure'
        self.senses['light']['maxvalue'] = lambda self: 200.0
        self.senses['light']['units'] = lambda self: "RAW"

        # location of sensors' hits:
        self.senses['light']['x'] = self.getIRXCoord
	self.senses['light']['y'] = self.getIRYCoord
	self.senses['light']['z'] = lambda self, pos: 0.25
	self.senses['light']['value'] = self.getLightRange
        #; self.senseData['light'][x]
	self.senses['light']['flag'] = self.getIRFlag

	# location of origin of sensors:
        self.senses['light']['ox'] = self.light_ox
	self.senses['light']['oy'] = self.light_oy
	self.senses['light']['oz'] = lambda self, pos: 0.25 # meters
	self.senses['light']['th'] = self.light_th
        # in radians:
        self.senses['light']['arc'] = lambda self, pos, \
                                      x = (5 * math.pi / 180) : x

        console.log(console.INFO,'khepera sense drivers loaded')

        self.controls['move'] = self._move
        self.controls['accelerate'] = self.accelerate
        self.controls['translate'] = self.translate
        self.controls['rotate'] = self.rotate
        self.controls['update'] = self.update 
        self.controls['localize'] = self.localize

        console.log(console.INFO,'khepera control drivers loaded')
        self.SanityCheck()

	self.update() # Khepera_UpdatePosition(self.dev)
        self.inform("Done loading robot.")

    def _draw(self, options, renderer): # overloaded from robot
        #self.setLocation(self.senses['robot']['x'], \
        #                 self.senses['robot']['y'], \
        #                 self.senses['robot']['z'], \
        #                 self.senses['robot']['thr'] )
        renderer.xformPush()
        renderer.color((0, 0, 1))
        #print "position: (", self.get('robot', 'x'), ",",  \
        #      self.get('robot', 'y'), ")"

        #renderer.xformXlate((self.get('robot', 'x'), \
        #                     self.get('robot','y'), \
        #                     self.get('robot','z')))
        renderer.xformRotate(self.get('robot', 'th'), (0, 0, 1))

        renderer.xformXlate(( 0, 0, .09))
        renderer.torus(.12, .12, 12, 24)

        renderer.color((.5, .5, .5))

        # wheel 1
        renderer.xformPush()
        renderer.xformXlate((0, .18, 0))
        renderer.xformRotate(90, (1, 0, 0))
        renderer.torus(.06, .06, 12, 24)
        renderer.xformPop()

        # wheel 2
        renderer.xformPush()
        renderer.xformXlate((0, -.18, 0))
        renderer.xformRotate(90, (1, 0, 0))
        renderer.torus(.06, .06, 12, 24)
        renderer.xformPop()

        # IR
        renderer.xformPush()
        renderer.color((0, 0, .7))
        for i in range(self.get('ir', 'count')):
            y1, x1, z1 = -self.get('ir', 'x', i), \
                         -self.get('ir', 'y', i), \
                         self.get('ir', 'z', i)
            y2, x2, z2 = -self.get('ir', 'ox', i), \
                         -self.get('ir', 'oy', i), \
                         self.get('ir', 'oz', i)
            x2, y2, z2 = 0, 0, 0
            arc    = self.get('ir', 'arc', i) # in radians
            renderer.ray((x1, y1, z1), (x2, y2, z2), arc)

        renderer.xformPop()        

        # end of robot
        renderer.xformPop()

    def getOptions(self): # overload 
        pass

    def connect(self):
        pass
        #Khepera_Localize(0.0, 0.0, 0.0)

    def disconnect(self):
        self.stop()

    def sendMsg(self, msg, data = '', type = 'i'):
        # Fix: this needs to be two processes: one write Queue
        # and one read Queue.
        #print "SENDING:", msg
        tries = 0
        done = 0
        while tries < 5 and not done:
            try:
                self.sc.writeline(msg)
                retval = self.sc.readline() # 1 = block till we get something
                if retval[0].upper() == msg[0]:
                    done = 1
                else:
                    tries += 1
            except:
                tries += 1
        if done == 0:
            print "khepera serial read/write error..."
            return
        if data:
            lines = string.split(retval, "\r\n")
            for line in lines:
                if not line == '':
                    # print "processing line:", line
                    irs = string.split(line, ",")
                    irs = irs[1:]
                    #print "RECEIVE data:", data, retval
                    try:
                        self.senseData[data] = array.array(type, map(int, irs))
                    except:
                        print "khepera packet error: type=", data, "vals=", irs
        
    def update(self):
        self.sendMsg('N', 'ir')     # proximity
        self.sendMsg('O', 'light')  # ambient light
        self.sendMsg('H', 'position')
        self.sendMsg('E', 'speed')

    def isStall(self, dev):
        return 0

    def getX(self, dev):
        return 0.0
    
    def getY(self, dev):
        return 0.0
    
    def getZ(self, dev):
        return 0
    
    def getTh(self, dev):
        return 0
    
    def getThr(self, dev):
        return 0
    
    def getIRXCoord(self, dev, pos):
        return 0

    def light_oy(self, dev, pos):
        # in mm
        if pos == 0:
            return 10.0
        elif pos == 1:
            return 20.0
        elif pos == 2:
            return 30.0
        elif pos == 3:
            return 30.0 
        elif pos == 4:
            return 20.0
        elif pos == 5:
            return 10.0
        elif pos == 6:
            return -30.0
        elif pos == 7:
            return -30.0

    def light_ox(self, dev, pos):
        # in mm
        if pos == 0:
            return 30.0
        elif pos == 1:
            return 25.0
        elif pos == 2:
            return 10.0
        elif pos == 3:
            return -10.0 
        elif pos == 4:
            return -25.0
        elif pos == 5:
            return -30.0
        elif pos == 6:
            return -10.0
        elif pos == 7:
            return 10.0

    def light_thd(self, dev, pos):
        if pos == 0:
            return 90.0
        elif pos == 1:
            return 45.0
        elif pos == 2:
            return 0.0
        elif pos == 3:
            return 0.0 
        elif pos == 4:
            return -45.0
        elif pos == 5:
            return -90.0
        elif pos == 6:
            return -180.0
        elif pos == 7:
            return 180.0

    def getIRRange(self, dev, pos):
        raw = self.senseData['ir'][pos]
        mm = min(max(((1023.0 - raw) / 1023.0) * 60.0, 0.0),
                 self.senses['ir']['maxvalue'](dev))
        if self.senses['ir']['units'](dev) == "ROBOTS":
            return mm / 55.0 # khepera is 55mm diameter
        elif self.senses['ir']['units'](dev) == "MM":
            return mm
        elif self.senses['ir']['units'](dev) == "RAW":
            return raw 
        elif self.senses['ir']['units'](dev) == "CM":
            return mm / 10.0 # cm
        elif self.senses['ir']['units'](dev) == "METERS":
            return mm / 100.0 # meters
        elif self.senses['ir']['units'](dev) == "SCALED":
            return mm / self.senses['ir']['maxvalue'](dev)
        else:
            raise 'InvalidType', "IR units are set to invalid type"

    def getLightRange(self, dev, pos):
        raw = self.senseData['light'][pos]
        mm = min(max((raw / 511.0) * 200.0, 0.0),
                 self.senses['light']['maxvalue'](dev))
        if self.senses['light']['units'](dev) == "ROBOTS":
            return mm / 55.0 # khepera is 55mm diameter
        elif self.senses['light']['units'](dev) == "MM":
            return mm
        elif self.senses['light']['units'](dev) == "RAW":
            return raw 
        elif self.senses['light']['units'](dev) == "CM":
            return mm / 10.0 # cm
        elif self.senses['light']['units'](dev) == "METERS":
            return mm / 100.0 # meters
        elif self.senses['light']['units'](dev) == "SCALED":
            return mm / self.senses['light']['maxvalue'](dev)
        else:
            raise 'InvalidType', "Light units are set to invalid type"

    def getIRRangeAll(self, dev):
        vector = [0] * self.get('ir', 'count')
        for i in range(self.get('ir', 'count')):
            vector[i] = self.getIRRange(dev, i)
        return vector

    def getIRFlag(self, dev, pos):
        return 0

    def getIRYCoord(self, dev, pos):
        return 0
    
    def light_th(self, dev, pos):
        return self.light_thd(dev, pos) / 180.0 * math.pi
    
    def move(self, trans, rotate):
        self._move(self, trans, rotate)
        self.update()

    def adjustSpeed(dev):
        # This will send new motor commands based on the
        # robot's lastTranslate and lastRotate settings.
        # Khepera has differential drive, so compute each
        # side motor commands:
        left  = int((dev.lastTranslate * dev.translateFactor - \
                     dev.lastRotate * dev.rotateFactor))
        right  = int((dev.lastTranslate * dev.translateFactor + \
                      dev.lastRotate * dev.rotateFactor))
        # FIX: add acceleration, and assume that adjustSpeed
        # is bing continuously called.
        dev.sendMsg('D,%i,%i' % (left, right))
        
    def _move(self, dev, trans, rotate):
        self.lastTranslate = trans
        self.lastRotate = rotate
        self.adjustSpeed()

    def accelerate(self, trans, rotate): # incr
        self.lastTranslate += trans
        self.lastRotate += rotate
        # FIX: do min/max here
        self.adjustSpeed()
        
    def translate(dev, value):
        dev.lastTranslate = value
        dev.adjustSpeed()
    
    def rotate(dev, value):
        dev.lastRotate = value
        dev.adjustSpeed()
    
    def localize(self):
        pass
    
if __name__ == '__main__':
    x = KheperaRobot()
    x.update()
    x.GetMin()
