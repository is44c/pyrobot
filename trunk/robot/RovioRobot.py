# Pyrobot interface for Rovio robot.
#Ryan Walker Swarthmore College June 2009
#Based on pyrobot interface and rovio text-based interface by Del Rudolph
# see http://www.gnu.org/copyleft/gpl.html for latest license

from pyrobot.robot import Robot
from pyrobot.robot.device import *

import urllib2
import urllib
import time
import threading, Queue
import subprocess, os
import cStringIO



class Rovio(Robot):
    def __init__(self, url, user='adminUsername', password='adminPassword'):
      Robot.__init__(self)

      # an Admin user account on Rovio. Needs to be Admin for all functions to work!
      self.theurl = url
      self.username = user
      self.password = password

      self.pingUrl()

      # said Admin user's password
      self.passman = urllib2.HTTPPasswordMgrWithDefaultRealm()
      self.passman.add_password(None, self.theurl, self.username, self.password)
      self.authhandler = urllib2.HTTPBasicAuthHandler(self.passman)
      self.opener = urllib2.build_opener(self.authhandler)
      urllib2.install_opener(self.opener)
      self.light = '0'
      # set up some globals
      self.battList = [126,126,126,126,126]
      self.emailok = 0
      self.head = 1
      self.obstacle = 0

      self.lock = threading.Lock()

    
    def connect(self): pass 
    def disconnect(self): pass

    def headPos(self, inpos):
      if inpos == self.head: 
        return
      data = ""
      # call thing to lower head /rev.cgi?Cmd=nav&action=18&drive=[down=12,mid=13,up=11]
      if inpos == 1 and self.head != '1':
          data = "12"
          self.head = "1"
      if inpos == 2 and self.head != '2':
          data = "13"
          self.head = "2"
      if inpos == 3 and self.head != '3':
          data = "11"
          self.head = "3"
      i = 0
      self.SendRequest("Cmd=nav&action=18&drive="+data+"")
      time.sleep(1)

    def Light(self):
      if self.light == "1":
          self.light = "0"
      else:
          self.light = "1"
      self.SendRequest("Cmd=nav&action=19&LIGHT="+self.light+"")

    
    def move(self, translate, rotate):
      self.translate(translate)
      self.rotate(rotate)
    
    def translate(self, val):
      if val != 0:
        speed = int((abs(val)+.19)*5)
        if val < 0:
          command = 2
        else:
          command = 1
        self.Drive(command, speed)
    
    def rotate(self, val):
      if val != 0:
        speed = 10-((abs(val)*10)-1)
        if speed > 10:
          speed = 10
        if val < 0:
          rotate = 6
        elif val > 0:
          rotate = 5
        self.Drive(rotate, speed)
          
    def strafe(self, val):
      if val != 0:
        if val < 0:
          command = 3
        else:
          command = 4
        self.Drive(command, 1)
    
    def startDeviceBuiltin(self, item):
      pass

    
    def update(self):
      self.updateStats()
    
    def localize(self, x = 0, y = 0, th = 0):
      print "need to override LOCALIZE in robot"
    
    def Drive(self,command, speed):
      self.lock.acquire()
      tRevUrl = "http://"+self.theurl+"/rev.cgi"
      mydata = "Cmd=nav&action=18&drive="+str(command)+"&speed="+str(speed)+""
      myreq = urllib2.Request(tRevUrl, mydata)
      urllib2.urlopen(myreq)
      time.sleep(0.15)
      self.lock.release()
    
    def stop(self):
      self.SendRequest("Cmd=nav&action=17")


    def SendRequest(self, indata, myreturn=0):
      self.lock.acquire()
      if indata == 'SendMail':
          thefile = '/SendMail.cgi'
          thedata = ""
      else:
          thefile = "/rev.cgi"
          thedata = indata

      req = urllib2.Request("http://"+self.theurl+thefile, thedata)
      handle = urllib2.urlopen(req)
      self.lock.release()
      if myreturn == 1:
          return handle.read()

    def updateStats(self):
    # batt 100=dead in the water, <106 go home, 127 full
    # wifi 0-254
    # nav_ss 0-65535, <5000 no signal, >47000 strong signal
        #Cmd = nav
        #responses = 0|x=-1339|y=-5592|theta=-1.953|room=0|ss=8263
        #|beacon=0|beacon_x=0|next_room=9|next_room_ss=38
        #|state=0|resistance=0|sm=15|pp=0|flags=0005
        #|brightness=6|resolution=3|video_compression=1|frame_rate=20
        #|privilege=0|user_check=1|speaker_volume=15|mic_volume=17
        #|wifi_ss=233|show_time=0|ddns_state=0|email_state=0
        #|battery=126|charging=80|head_position=203|ac_freq=2
    # makeProgBar(width,min,max,val)
      stats = {}
      statstr = self.SendRequest("Cmd=nav&action=1", 1)
      statstr = statstr.replace("Cmd = nav\nresponses = 0|", '')
      statstr = statstr.replace("\n", "")
      for item in statstr.split('|'):
          a,b = item.split('=')
          stats[a] = b
      #obstacle check
      self.obstacle = int(stats['flags'])
      if self.obstacle == 5:
        self.obstacle = 0
      elif self.obstacle == 7:
        self.obstacle = 1
        
      self.emailok = int(stats['email_state'])

      i = 0
      battavg = 0
      del self.battList[0]
      self.battList.append(int(stats['battery']))
      for item in self.battList: i += item
      battavg = i / 5

      if battavg < 108:
        battPercent = 27.0/(battavg-100)
        print "LOW BATTERY WARNING. Battery at: "+battPercent


      # Going Home state doesn't seem to be used?
      state = ('  Roaming  ', 'Going Home', '  Docking  ', '           ', 'No Connection')

      hp = int(stats['head_position'])
      if hp > 195 and hp < 205:
        self.head = 1
      if hp > 130 and hp < 150:
        self.head = 2
      if hp > 60 and hp < 70:
        self.head = 3

    def emailImage(self):
      if emailok == 1:
        # send email
        self.SendRequest("SendMail")

    def checkObstacle(self):
      #returns status of front obstacle detector
      #rougly 2' range.  0 = clear; 1=obstructed
      return self.obstacle

    def wave(self):
      self.headPos(1)
      self.headPos(3)
      self.headPos(1)

    def goHome(self):
      self.SendRequest("Cmd=nav&action=13")

    def imageDump(self):
      self.lock.acquire()
      urllib.urlretrieve("http://"+self.theurl+"/Jpeg/CamImg1234.jpg", "test.jpg")
      print "pic retrieved"
      self.lock.release()

    def imageGrab(self):
      self.lock.acquire()
      pic = urllib.urlopen("http://"+self.theurl+"/Jpeg/CamImg1234.jpg")

    def pingUrl(self):
      os.system('ping -c2 ' + self.theurl)











def INIT():
    return Rovio('10.0.0.9')

if __name__=='__main__':
  x = Rovio('10.0.0.9')
  x.update()
  x.move(1,0)