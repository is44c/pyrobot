#
# This file contains the class that represents a computer controlled
# physical agent (robot). A robot is a bunch of drivers that offer
# senses and controllers
#
# - stephen -
#
# -----------------------------
# Main interfaces:
# .get()             - interface to robot, sensors

import pyro.gui.console as console
from pyro.geometry import Polar, distance
import math, string, types

# Units of measure for sense, map, and motors:
# -------------------------------------------
# ROBOTS - unit is given interms of robot's diameter
# METERS - meters
# CM     - centimeters
# MM     - millimeters
# SCALED - scaled [-1,1]
# RAW    - right from the sensor

def serviceDirectoryFormat(serviceDict, retdict = 1):
    """
    Takes a service directory dictionary and makes it presentable for viewing.

    Also takes a flag to indicate if it should return a dictionary (for * listings)
    or a list (for regular directory content listing).
    
    The function does two things:
      1. adds a trailing slash to those entries that point to more things
      2. replaces the objects that they point to with None, when returning a dictionary
    """
    if retdict:
        retval = {}
    else:
        retval = []
    for keyword in serviceDict:
        if type(serviceDict[keyword]) == types.InstanceType:
            if retdict:
                retval[keyword + "/"] = None
            else:
                retval.append( keyword + "/")
        else:
            if retdict:
                retval[keyword] = serviceDict[keyword]
            else:
                retval.append( keyword )
    return retval

def expand(part):
    """
    Takes a part of a path and parses it for simple syntax. Expands:
    -  inclusive range
    :  slice range
    ,  AND
    """
    retvals = []
    if type(part) == type(1):
        return [part]
    elif isinstance(part, (type((1,)), type([1,]))):
        return part
    else:
        if type(part) != type(""):
            raise AttributeError, "path part should be an int, list, tuple, or string keyword"
    # else, it had better be a string
    if part.find(",") >= 0:
        subparts = map(string.strip, part.split(","))
    else:
        subparts = [string.strip(part),]
    for s in subparts:
        if s.count(":") == 1:
            rangeVals = map(string.strip, s.split(":"))
            if rangeVals[0].isdigit() and rangeVals[0].isdigit():
                retvals.extend(range(int(rangeVals[0]), int(rangeVals[1])))
            else:
                if s.isdigit():
                    retvals.append( int(s) )
                else:
                    retvals.append( s )
        elif s.count("-") == 1:
            rangeVals = map(string.strip, s.split("-"))
            print "rangeVals", rangeVals
            if rangeVals[0].isdigit() and rangeVals[0].isdigit():
                retvals.extend(range(int(rangeVals[0]), int(rangeVals[1]) + 1))
            else:
                if s.isdigit():
                    retvals.append( int(s) )
                else:
                    retvals.append( s )
        else:
            if s.isdigit():
                retvals.append( int(s) )
            else:
                retvals.append( s )
    if len(retvals) == 1:
        return retvals[0]
    else:
        return retvals
                        
class Robot:
    """
    this be the one.
    """
    def __init__(self, **kwargs):
        """
        if you extend Robot please call this function!
        If you need to initialize things, call setup()
        """
        self.service = {}
        self.supports = {}
        self.data = {}
        self.dataFunc = {}
        self.service["robot"] = self
        # user init:
        self.setup(**kwargs)

    #def __getattr__(self, path):
    #    #if path[:2] != "__":
    #    return self.get(path)

    def __repr__(self):
        retval = 'Robot:\n---------------------------------\n'
        for item in self.get("robot/"):
            if item[-1] == "/": # more things below this
                retval += "    %s\n" % item
            else:
                retval += "    %s = %s\n" % (item, self.get("robot/%s" % item))
        return retval

    def disconnect(self):
        console.log(console.WARNING, "need to override DISCONNECT in robot")

    def localize(self, x = 0, y = 0, th = 0):
        console.log(console.WARNING, "need to override LOCALIZE in robot")

    def inform(self, msg):
        console.log(console.INFO, msg)
        
    def _get(self, pathList):
        if len(pathList) == 0:
            tmp = self.data.copy()
            tmp.update( self.dataFunc )
            return serviceDirectoryFormat(tmp, 0)
        key = pathList[0]
        args = pathList[1:]
        if key in self.data:
            return self.data[key]
        elif key in self.dataFunc:
            return self.dataFunc[key]._get(args)
        if key == '*':
            if args != []:
                raise AttributeError, "wildcard feature not implemented in directory middle"
            tmp = self.data.copy()
            tmp.update( self.dataFunc )
            return serviceDirectoryFormat(tmp, 1) 
        else:
            raise AttributeError, "no such directory item '%s'" % key

    def get(self, device = "", *args):
	"""
	this is designed to be the main interface to the robot
	and its parts. There is one assumed piece, self.dev that
	is the actual pointer to the robot device

        Device names should not contain slashes, commas, dashes, or colons, nor have
        spaces as part of their names (actually spaces inside their names is fine).
	"""
        path = device.split("/")
        # remove extra slashes
        while path.count("") > 0:
            path.remove("")
        path.extend( args )
        # parse path parts for dashes, colons, and commas
        finalPath = []
        for part in path:
            finalPath.append(expand( part ) )
        if len(finalPath) == 0:
            return serviceDirectoryFormat(self.service, 0) # toplevel in service
        elif finalPath[0] in self.service:
            # pass the command down to robot
            return self.service[finalPath[0]]._get(finalPath[1:])
        elif finalPath[0] == '*':
            return serviceDirectoryFormat(self.service, 1)
        else:
            raise AttributeError, "'%s' is not a service directory of robot" % finalPath[0]

    def _set(self, pathList, value):
        key = pathList[0]
        args = pathList[1:]
        if key in self.data:
            self.data[key] = value
        elif key in self.dataFunc:
            return self.dataFunc[key]._set(args, value)
        else:
            raise AttributeError, "no setable directory item '%s'" % key
        
    def set(self, device, value):
	"""
	"""
        path = device.split("/")
        # remove extra slashes
        while path.count("") > 0:
            path.remove("")
        if path[0] in self.service: # toplevel in service
            # pass the command down to robot
            return self.service[path[0]]._set(path[1:], value)
        else:
            raise AttributeError, "'%s' is not a service directory of robot" % path[0]

    def step(self, dir):
        if dir == 'L':
            self.rotate(.2)
        elif dir == 'R':
            self.rotate(-.2)
        elif dir == 'B':
            self.translate(-.2)
        elif dir == 'F':
            self.translate(0.2)
        elif dir == 'ST':
            self.translate(0.0)
        elif dir == 'SR':
            self.rotate(0.0)

    def motors(self, leftValue, rightValue):
        """
        Move function that takes desired motor values
        and converts to trans and rotate.
        """
        trans = (rightValue + leftValue) / 2.0
        rotate = (rightValue - leftValue) / 2.0
        self.move(trans, rotate)
        
    def stop(self):
        self.move(0, 0)

    def _update(self):
        for service in self.getServices():
            if self.getService(service).active:
                self.getService(service).updateService()

    # Need to define these:

    def move(self, translate, rotate):
        pass

    def translate(self, val):
        pass

    def rotate(self, val):
        pass

    def update(self):
        """
        This is a common control that isn't an act, so we pull it out
        here.
        """
        pass

    def enableMotors(self):
        pass

    def disableMotors(self):
        pass

    # -------------------- Angle and Distance functions:

    def getAngleToAngle(self, phi): # phi is in radians
        '''
        Given an angle in radians (0 front, to left to PI), what is the
        shortest way to turn there?  returns -PI to PI, neg to right,
        to use with turning
        '''
        theta = self.get("/robot/thr/")
        if (phi > theta):  # turn left
            phi = phi - theta;
        else: # // turn right
            phi = (theta - phi) * -1.0;
        if (phi > math.pi): # // oops, shorter to turn other direction
            phi = (2 * math.pi - phi) * -1.0;
        if (phi < -math.pi): #// oops, shorter to turn other direction
            phi = (2 * math.pi + phi);
        return phi

    def getAngleToPoint(self, x, y):
        return self.getAngleToPoints(x, y, self.get('robot/x'), self.get('robot/y'))
                                     
    def getAngleToPoints(self, x1, y1, x2, y2):
        p = Polar()
        p.setCartesian(x1 - x2, y1 - y2) # range pi to -pi
        if (p.t < 0.0):
            phi = p.t + 2 * math.pi # 0 to pi to left; 0 to -pi to right
        else:
            phi = p.t;
        return self.getAngleToAngle(phi)

    def getDistanceToPoint(self, x, y):
        return distance(x, y, self.get('robot/x'), self.get('robot/y'))

    # ------------------------- Service functions:

    def startServices(self, item):
        """ Alias for startService() """
        return self.startService(item)
        
    def startService(self, item):
        """ Load a service: dict, list, or name or filename """
        import pyro.system as system
        import os
        # Item can be: dict, list, or string. string can be name or filename
        if type(item) == type({}):
            retval = []
            for service in item.keys():
                console.log(console.INFO,"Loading service '%s'..." % service)
                if self.service.has_key(service):
                    print "Service is already running: '%s'" % service
                    retval.append( self.service[service] )
                else:
                    retval.append(item[service].startService())
                if item[service].getServiceState() == "started":
                    self.service[service] = item[service]
                    if service not in self.senses.keys():
                        self.senses[service] = self.service[service]
                else:
                    print "service '%s' not available" % service
                    retval.append( None )
            return retval
        elif type(item) == type([0,]) or \
             type(item) == type((0,)):
            # list of services
            retval = []
            for service in item:
                retval.append(self.startService(service))
            return retval
        elif self.supportsService(item): # built-in name
            if self.service.has_key(item):
                print "Service is already running: '%s'" % item
                return [self.service[item]]
            console.log(console.INFO,"Loading service '%s'..." % item)
            retval = self.supports[item].startService()
            if self.supports[item].getServiceState() == "started":
                self.service[item] = self.supports[item]
                self.senses[item] = self.service[item]
            else:
                print "service '%s' not available" % item
            return [retval]
        else: # from a file
            file = item
            if file[-3:] != '.py':
                file = file + '.py'
            if system.file_exists(file):
                return self.startService( system.loadINIT(file, self) )
            elif system.file_exists(os.getenv('PYRO') + \
                                    '/plugins/services/' + file): 
                return self.startService( system.loadINIT(os.getenv('PYRO') + \
                                                   '/plugins/services/'+ \
                                                   file, self))
            else:
                print 'Service not found: ' + file
                return []

    def stopService(self, item):
        self.getService(self, item).stopService()

    def supportsService(self, item):
        return self.supports.has_key(item)

    def getService(self, item):
        if self.service.has_key(item):
            return self.service[item]
        else:
            raise AttributeError, "unknown service '%s'" % item

    def getServiceDevice(self, item):
        if self.service.has_key(item):
            return self.service[item].dev
        else:
            raise AttributeError, "unknown service '%s'" % item

    def getServiceData(self, item, *args):
        if self.service.has_key(item):
            return self.service[item].getServiceData(*args)
        else:
            raise AttributeError, "unknown service '%s'" % item

    def getServices(self):
        #return self.service.keys()
        return []

    def getSupportedServices(self):
        return self.supports.keys()

    def hasService(self, item):
        return self.service.has_key(item)

    def removeService(self, item):
        self.service[item].visible = 0
        self.service[item].active = 0
        self.service[item].destroy()
        del self.service[item]

    # Message interface:

    def sendMessage(self, message):
        raise AttributeError, "no send message interface"

    def getMessages(self):
        return []

    def setup(self, **kwargs):
        """
        Is called from __init__ so users don't have to call parent
        constructor and all the gory details.
        """
        pass

if __name__ == "__main__":
    r = Robot()
