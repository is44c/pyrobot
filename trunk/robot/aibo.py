"""
Ioana Butoi and Doug Blank
Aibo client commands for talking to the Tekkotsu servers
from Python.
"""

from pyro.robot import Robot
from pyro.robot.device import Device
from socket import *
import struct, time, sys, threading

def makeControlCommand(control, amt):
    # HEAD: control is tilt 't', pan 'p', or roll 'r'
    # WALK: control is forward 'f', strafe 's', and rotate 'r'
    return struct.pack('<bf', ord(control), amt) 

class Listener:
    """
    A class for talking to ports on Aibo. If you want to read the data off
    in the background, give this thread to ListenerThread, below.
    """
    def __init__(self, port, host, protocol = "TCP"):
        self.port = port
        self.host = host
        self.protocol = protocol
        self.runConnect()
        #self.s.settimeout(0.1)

    def runConnect(self):
        print >> sys.stderr, "[",self.port,"] connecting ...",
        try:
            if self.protocol == "UDP":
                self.s = socket(AF_INET, SOCK_DGRAM) # udp socket
            elif self.protocol == "TCP":
                self.s = socket(AF_INET, SOCK_STREAM) # tcp socket
            done = 0
            while not done:
                try:
                    self.s.connect((self.host,self.port)) # connect to server
                    done = 1
                except KeyboardInterrupt:
                    print >> sys.stderr, "aborted!"
                    return
                except:
                    print >> sys.stderr, ".",
            print >> sys.stderr, "connected!"
        except IOError, e:
            print e

    def readUntil(self, stop = "\n"):
        retval = ""
        ch = self.s.recvfrom(1)[0]
        while ch != stop:
            retval += struct.unpack('c', ch)[0]
            ch = self.s.recvfrom(1)[0]
        return retval

    def read(self, bytes = 4, format = 'l', all=False):
        data = ""
        for i in range(bytes):
            data += self.s.recvfrom(1)[0]
        if all:
            return struct.unpack(format, data)
        else:
            return struct.unpack(format, data)[0]

    def write(self, message):
        retval = self.s.send(message)
        return retval

class ListenerThread(threading.Thread):
    """
    A thread class, for ports where Aibo feeds it to us
    as fast as we can eat em!
    """
    def __init__(self, listener, callback):
        """
        Constructor, setting initial variables
        """
        self.listener = listener
        self.callback = callback
        self._stopevent = threading.Event()
        self._sleepperiod = 0.01
        threading.Thread.__init__(self, name="ListenerThread")
        
    def run(self):
        """
        overload of threading.thread.run()
        main control loop
        """
        while not self._stopevent.isSet():
            self.callback(self.listener)
            self._stopevent.wait(self._sleepperiod)

    def join(self,timeout=None):
        """
        Stop the thread
        """
        self._stopevent.set()
        threading.Thread.join(self, timeout)

class AiboHeadDevice(Device):
    """
    Class for moving the Aibo Head unit as a Pan-Tilt-Zoom-Nod device.
    Does not implement zoom.
    """
    def __init__(self, robot):
        #Device.__init__(self, "ptz")
        self.robot = robot
        # Turn on head remote control if off:
        self.robot.setRemoteControl("Head Remote Control", "on")
        time.sleep(1) # pause for a second
        self.dev   = Listener(10052, self.robot.host) # head movement
        self.devData["supports"] = ["pan", "tilt", "roll"]
        self.notGetables.extend (["tilt", "pan", "zoom", "roll"])
        self.pose = [0, 0, 0, 0]

    def init(self):
        self.center()

    def postGet(self, keyword):
        if keyword == "pose":
            return self.pose

    def postSet(self, keyword):
        if keyword == "pose":
            self.setPose( self.devData[keyword] )
        elif keyword == "pan":
            self.pan( self.devData[keyword] )
        elif keyword == "tilt":
            self.tilt( self.devData[keyword] )
        elif keyword == "zoom":
            self.zoom( self.devData[keyword] )
        elif keyword == "roll":
            self.roll( self.devData[keyword] )
        return "ok"

    def setPose(self, *args):
        pan, tilt, zoom, roll = 0, 0, 0, 0
        if len(args) == 3:
            pan, tilt, zoom = args[0], args[1], args[2]
        elif len(args) == 4:
            pan, tilt, zoom, roll = args[0], args[1], args[2], args[3]
        elif len(args[0]) == 3:
            pan, tilt, zoom = args[0][0], args[0][1], args[0][2]
        elif len(args[0]) == 4:
            pan, tilt, zoom, roll = args[0][0], args[0][1], args[0][2], args[0][3]
        else:
            raise AttributeError, "setPose takes pan, tilt, zoom (ignored)[, and roll]"
        self.pan( pan )
        self.tilt( tilt )
        self.zoom( zoom )
        self.roll( roll )
        return "ok"

    def zoom(self, amt):
        return "ok"

    def tilt(self, amt):
        # tilt: 0 to -1 (straight ahead to down)
        # see HeadPointListener.java
        if amt > 0:
            amt = 0
        if amt < -1:
            amt = -1
        self.dev.write( makeControlCommand('t', amt))
        self.pose[1] = amt
        return "ok"

    def pan(self, amt):
        # pan: -1 to 1 (right to left)
        # see HeadPointListener.java
        if amt > 1:
            amt = 1
        if amt < -1:
            amt = -1
        self.dev.write( makeControlCommand('p', amt)) 
        self.pose[0] = amt
        return "ok"

    def roll(self, amt):
        # roll: 0 to 1 (straight ahead, to up (stretched))
        if amt < 0:
            amt = 0
        if amt > 1:
            amt = 1
        self.dev.write( makeControlCommand('r', amt))
        self.pose[3] = amt
        return "ok"

    def center(self):
        return self.setPose(0, 0, 0, 0)

class AiboRobot(Robot):
    """
    Class for an Aibo robot in Pyro. 
    """
    # TODO: put listeners in a dict, referenced by name
    # Look up port here:
    PORT = {"Head Remote Control": 10052,
            "Root Control": 10020,
            "Walk Remote Control": 10050, 
            "EStop Remote Control": 10053,
            "World State Serializer": 10031,
            }
    def __init__(self, host):
        Robot.__init__(self)
        self.host = host
        #---------------------------------------------------
        self.main_control     = Listener(10020,self.host) # menu controls
        self.main_control.s.send("!reset\n") # reset menu
        # --------------------------------------------------
        self.setRemoteControl("Walk Remote Control", "on")
        self.setRemoteControl("EStop Remote Control", "on")
        self.setRemoteControl("World State Serializer", "on")
        time.sleep(1) # let the servers get going...
        self.walk_control     = Listener(self.PORT["Walk Remote Control"],
                                         self.host)
        self.estop_control    = Listener(self.PORT["EStop Remote Control"],
                                         self.host) # stop control
        #wsjoints_port   =10031 # world state read sensors
        #wspids_port     =10032 # world state read pids        
        self.sensor_socket    = Listener(self.PORT["World State Serializer"],
                                         self.host) # sensors
        self.sensor_thread    = ListenerThread(self.sensor_socket, self.readWorldState)
        self.sensor_thread.start()
        #self.pid_socket       = Listener(10032, self.host) # sensors
        time.sleep(1) # let all of the servers get going...
        self.estop_control.s.send("start\n") # send "stop\n" to emergency stop the robot
        time.sleep(1) # let all of the servers get going...
        self.devData["builtinDevices"] = [ "ptz", "camera" ]
        #self.startDevice("ptz")
        #self.startDevice("camera")

        # Commands available on main_control (port 10020):
        # '!refresh' - redisplays the current control (handy on first connecting,
        #               or when other output has scrolled it off the screen)
        # '!reset' - return to the root control
        # '!next' - calls doNextItem() of the current control
        # '!prev' - calls doPrevItem() of the current control
        # '!select' - calls doSelect() of the current control
        # '!cancel' - calls doCancel() of the current control
        # '!msg text' - broadcasts text as a TextMsgEvent
        # '!root text ...' - calls takeInput(text) on the root control
        # '!cmd control' - calls takeInput("") on control
        # '!hello' - responds with 'hello\ncount\n' where count is the number of times
        #            '!hello' has been sent.  Good for detecting first connection after
        #            boot vs. a reconnect.
        # '!hilight [n1 [n2 [...]]]' - hilights zero, one, or more items in the menu
        # '!input text' - calls takeInput(text) on the currently hilighted control(s)
        # '!set section.key = value' - will be sent to Config::setValue(section,key,value)
        #  any text not beginning with ! - sent to takeInput() of the current control

    def setConfig(self, item, state):
        self.main_control.s.send("!set \"%s=%s\"\n" % (item, state))

    def setRemoteControl(self, item, state):
        # "Walk Remote Control"
        # could also use "!root "TekkotsuMon" %(item)"
        if state == "off":
            item = "#" + item
        self.main_control.s.send("!cmd \"%s\"\n" % item)

        # Main menu:
        # 0 Mode Switch - Contains the "major" apps, mutually exclusive selection
        # 1 Background Behaviors - Background daemons and monitors
        # 2 TekkotsuMon: Servers for GUIs
        # 3 Status Reports: Displays info about the runtime environment on the console
        # 4 File Access: Access/load files on the memory stick
        # 5 Walk Edit: Edit the walk parameters
        # 6 Posture Editor: Allows you to load, save, and numerically edit the posture
        # 7 Vision Pipeline: Start/Stop stages of the vision pipeline
        # 8 Shutdown?: Confirms decision to reboot or shut down
        # 9 Help: Recurses through the menu, prints name and description of each item

        # Option 2 from Main Menu, TekkotsuMon menu:
        # 0 RawCamServer: Forwards images from camera, port 10012
        # 1 SegCamServer: Forwards segmented images from camera, port 10012
        # 2 Head Remote Control: Listens to head control commands, port 10052
        # 3 Walk Remote Control: Listens to walk control commands, port 10050
        # 4 View WMVars: Brings up the WatchableMemory GUI on port 10061
        # 5 Watchable Memory Monitor: Bidirectional control communication, port 10061
        # 6 Aibo 3D: Listens to aibo3d control commands coming in from port 10051
        # 7 World State Serializer: Sends sensor information to port 10031
        #                           and current pid values to port 10032
        # 8 EStop Remote Control

    def update(self):
        self._update()

    def readWorldState(self, socket):
        """ Used as a callback in ListenerThread for sockets that produce data fast for us to read. """
        # read sensor/pid states:
        self.devData["ws_timestamp"] = socket.read(4, "l")
        # ---
        numPIDJoints = socket.read(4, "l")
        self.devData["numPIDJoints"] = numPIDJoints # ERS7: 18
        self.devData["positionRaw"] = socket.read(numPIDJoints * 4,
                                                  "<%df" % numPIDJoints,all=1)
        # ---
        numSensors = socket.read(4, "l") # ERS7: 8
        self.devData["numSensors"] = numSensors
        self.devData["sensorRaw"] = socket.read(numSensors * 4,
                                                "<%df" % numSensors,all=1)
        # ---
        numButtons = socket.read(4, "l") # ERS7: 6
        self.devData["numButtons"] = numButtons
        self.devData["buttonRaw"] = socket.read(numButtons * 4,
                                                "<%df" % numButtons,all=1)
        # --- same number as PID joints:             # ERS7: 18
        self.devData["dutyCycleRaw"] = socket.read(numPIDJoints * 4,
                                                   "<%df" % numPIDJoints,all=1)
                
    def getJoint(self, jointGroup, jointD1="", jointD2="", jointD3=""):
        """ Get position, dutyCycle of joint by name """
        legOffset = 0
        numLegs = 4
        jointsPerLeg = 3
        numLegJoints = numLegs*jointsPerLeg
        headOffset = legOffset+numLegJoints
        numHeadJoints = 3
        tailOffset = headOffset + numHeadJoints
        numTailJoints = 2
        mouthOffet = tailOffset + numTailJoints
        if jointGroup == "leg":
            if jointD2 == "front":
                offset = 0
            elif jointD2 == "back":
                offset = numLegs/2
            if jointD1 == "left":
                offset = legOffset + offset*jointsPerLeg
            elif jointD1 == "back":
                offset = legOffset + (offset+1)*jointsPerLeg
            if jointD3 == "rotator":
                # moves leg forward or backward along body
                pos = offset
            elif jointD3 == "elevator":
                # moves leg toward or away from body
                pos = offset + 1
            elif jointD3 == "knee":
                pos = offset + 2
        elif jointGroup == "head":
            if jointD1 == "tilt":
                pos = headOffset
            elif jointD1 == "pan":
                pos = headOffset +1
            elif jointD1 == "roll":
                pos = headOffset +2
            elif jointD1 == "nod":
                pos = headOffset+2
        elif jointGroup == "tail":
            if jointD1 == "tilt":
                pos = tailOffset
            elif jointD1 == "pan":
                pos = tailOffset +1
        elif jointGroup == "mouth":
            pos = mouthOffset
        else:
            AttributeError, "no such joint: '%s'" % jointGroup
        return self.devData["positionRaw"][pos], self.devData["dutyCycleRaw"][pos]

    def startDeviceBuiltin(self, item):
        if item == "ptz":
            return {"ptz": AiboHeadDevice(self)}
        elif item == "camera":
            return self.startDevice("AiboCamera")

    def connect(self):
        self.estop_control.s.send("start\n")

    def disconnect(self):
        self.estop_control.s.send("stop\n")

    def enableMotors(self):
        self.estop_control.s.send("start\n")

    def disableMotors(self):
        self.estop_control.s.send("stop\n")

    def rotate(self, amount):
        self.walk_control.write( makeControlCommand('r', amount)) 

    def translate(self, amount):
        self.walk_control.write( makeControlCommand('t', amount)) 

    def strafe(self, amount):
        # strafe (side-to-side) -1 to 1 :(right to left)
        self.walk_control.write( makeControlCommand('s', amount)) 
        
    def move(self, translate, rotate):
        # forward: -1 to 1 (backward to forward)
        # rotate: -1 to 1 (right to left)
        self.walk_control.write( makeControlCommand('f', translate)) 
        self.walk_control.write( makeControlCommand('r', rotate))

    def destroy(self):
        self.sensor_thread.join()
        Robot.destroy(self)

    def setWalk(self, value):
        # If you change the walk, then you have to reconnect
        # back onto the walk server
        # PACE.PRM TIGER.PRM (crawl) WALK.PRM
        code = {"pace":0, "walk":1, "crawl":2}
        self.walk_control.s.close()
        self.main_control.s.send("!reset\n")
        self.main_control.s.send("5\n") # walk editor
        self.main_control.s.send("9\n") # load walk
        self.main_control.s.send("%d\n" % code[value]) # walk number
        self.main_control.s.send("0\n") # stop walk socket
        self.main_control.s.send("0\n") # start walk socket
        self.walk_control     = Listener(10050, self.host) # walk command

#TODO:

# 1. How to read sensors? Infrared, Touch, joint positions
# 2. How to change gait? Running? Walking? On knees?
# 3. Make a "virtual range sensor" from vision

# Aibo 3D: Listens to aibo3d control commands coming in from port 10051

# World State Serializer: Sends sensor information to port 10031 and
# current pid values to port 10032
