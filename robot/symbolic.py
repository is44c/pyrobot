"""
The client robot connection programs for the PyrobotSimulator
non-symbolic robots.

(c) 2005, PyrobRobotics.org. Licenced under the GNU GPL.
"""

__author__ = "Douglas Blank <dblank@brynmawr.edu>"
__version__ = "$Revision$"

import socket, pickle, threading, random, time
from pyrobot.robot import Robot
from pyrobot.robot.device import Device, SensorValue

class SimulationDevice(Device):
	def __init__(self, robot):
		Device.__init__(self, "simulation")
		self._dev = robot
		self.startDevice()
	def setPose(self, name, x = 0, y = 0, thr = 0):
		self._dev.move("a_%s_%f_%f_%f" % (name, x, y, thr))
		self._dev.localize(0,0,0)
		return "ok"
	def getPose(self, name, x = 0, y = 0, thr = 0):
		retval = self._dev.move("c_%s" % (name, ))
		return retval

class RangeSimDevice(Device):
	def __init__(self, name, index, robot, geometry, groups):
		Device.__init__(self, name)
		self._geometry = geometry
		self.groups = groups
		self.startDevice()
		self._dev = robot
		self.index = index
		self.maxvalueraw = geometry[2]
		self.rawunits = "M"
		self.units = "ROBOTS"
		self.radius = robot.radius
		self.count = len(self)
		self._noise = 0.05
		
	def __len__(self):
		return len(self._geometry[0])
	def getSensorValue(self, pos):
		try:
			v = self._dev.__dict__["%s_%d" % (self.type, self.index)][pos]
		except:
			v = 0.0
		return SensorValue(self, v, pos,
				   (self._geometry[0][pos][0], # x in meters
				    self._geometry[0][pos][1], # y
				    0.03,                    # z
				    self._geometry[0][pos][2], # th
				    self._geometry[1]),        # arc rads
				   noise=self._noise
				   )

class LightSimDevice(RangeSimDevice):
	def __init__(self, *args, **kwargs):
		RangeSimDevice.__init__(self, *args, **kwargs)
		self.units = "SCALED"
	def _getRgb(self):
		retval = []
		for i in range(len(self)):
			retval.append( self.getSensorValue(i).rgb )
		return retval
	def getSensorValue(self, pos):
		retval = RangeSimDevice.getSensorValue(self, pos)
		retval.rgb = self._dev.move("f_%d_%d" % (self.index, pos)) # rgb
		return retval
	rgb = property(_getRgb)

class BulbSimDevice(Device):
	def __init__(self, robot):
		Device.__init__(self)
		self.type = "bulb"
		self._dev = robot
	def setBrightness(self, value):
		return self._dev.move("h_%f" % value)

class TCPRobot(Robot):
	"""
	A simple TCP-based socket robot for talking to PyrobotSimulator.
	"""
	BUFSIZE = 4096 # 2048 # 1024
	def __init__(self, host, port):
		Robot.__init__(self)
		self.lock = threading.Lock()
		# Set the socket parameters
		self.host = host
		self.port = port
		self.addr = (host, port)
		self.type = "Pyrobot"
		# Create socket
		self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		try:
			self.socket.settimeout(1)
		except:
			print "WARN: entering deadlock zone; upgrade to Python 2.3 to avoid"
		done = 0
		while not done:
			try:
				self.socket.connect( self.addr )
				done = 1
			except:
				print "Waiting on PyrobotSimulator..."
				time.sleep(1)
		self.connectionNum = self.getItem("connectionNum:%d" % self.port)
		self.radius = self.getItem("radius")
		self.properties = self.getItem("properties")
		self.builtinDevices = self.getItem("builtinDevices")
		self.builtinDevices.append("simulation")
		self.supportedFeatures = self.getItem("supportedFeatures")
		self.name = self.getItem("name")
		self.id   = self.connectionNum
		for dev in self.builtinDevices:
			d = self.startDevice(dev)
			if dev in ["sonar", "laser"]:
				self.range = d

	def localize(self, x = 0, y = 0, th = 0):
		pass

	def update(self):
		for i in self.properties:
			self.__dict__[i] = self.getItem(i)
		self.updateDevices()
		
	def getItem(self, item):
		return self.move(item)

	def eat(self, amt):
		return self.move("e_%f" % (amt))

	def play(self, item):
		return self.move(item)

	def tell(self, item):
		return self.move(item)

	def ask(self, item):
		return self.move(item)

	def _moveDir(self, dir):
		if dir == 'L':
			self.move("left")
		elif dir == 'R':
			self.move("right")
		elif dir == 'F':
			self.move("forward")
		elif dir == 'B':
			self.move("back")
	def localize(self, x = 0, y = 0, thr = 0):
		return self.move("b_%f_%f_%f" % (x, y, thr))

	def startDeviceBuiltin(self, name, index = 0):
		if name == "simulation":
			return {"simulation": SimulationDevice(self)}
		elif name == "bulb":
			self.move("s_%s_%d" % (name, index))
			return {name: BulbSimDevice(self)}
		elif name == "light":
			self.properties.append("%s_%d" % (name, index))
			self.move("s_%s_%d" % (name, index))
			geometry = self.move("g_%s_%d" % (name, index))
			groups = self.move("r_%s_%d" % (name, index))
			return {name: LightSimDevice(name, index, self, geometry, groups)}
		else:
			self.properties.append("%s_%d" % (name, index))
			self.move("s_%s_%d" % (name, index))
			geometry = self.move("g_%s_%d" % (name, index))
			groups = self.move("r_%s_%d" % (name, index))
			return {name: RangeSimDevice(name, index, self, geometry, groups)}

	def translate(self, value):
		self.move("t_%f" % value)

	def rotate(self, value):
		self.move("o_%f" % value)

	def move(self, message, other = None):
		self.lock.acquire()
		if type(message) in [type(1), type(1.)] and type(other) in [type(1), type(1.)]:
			message = "m_%.2f_%.2f" % (message, other)
			other = None
		exp = None
		if self.socket == 0: return "not connected"
		if other != None: return # rotate,translate command ignored
		if message == "quit" or message == "exit" or message == "end" or message == "disconnect":
			self.socket.sendto(message, self.addr)
			self.socket.close()
			self.socket = 0
			self.lock.release()
			return "ok"
		else:
			self.socket.sendto(message, self.addr)
			try:
				retval, addr = self.socket.recvfrom(self.BUFSIZE)
			except:
				retval = ""
			retval = retval.strip()
			try:
				exp = pickle.loads( retval )
			except:
				exp = retval
		self.lock.release()
		return exp

	def disconnect(self):
		if self.connectionNum == 0: # the main one, let's close up!
			# Close socket
			self.getItem("quit")
		else:
			self.getItem("disconnect")

