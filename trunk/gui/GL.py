from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.Tk import *

from pyro.gui import *
from pyro.gui.widgets.TKwidgets import *
from pyro.system.version import *
from pyro.engine import *
import pyro.system as system
from pyro.gui.drawable import *
from pyro.gui.renderer.gl import *
from pyro.gui.renderer.streams import *

from time import time
from pyro.gui.widgets.TKwidgets import *

# A GL gui

class GLgui(gui): 
   def __init__(self, engine, width = 400, height = 400, db = 1, depth = 1,
                mode = 1): # mode = 1 opengl, 0 = tk
      gui.__init__(self, 'GL gui', {}, engine)
      self.graphicsMode = mode
      self.width = width
      self.height = height
      self.genlist = 0
      self.frame = Frame()
      self.frame.pack(side = 'top')
      self.windowBrain = 0
      self.windowRobot = 0
      self.lastRun = 0

      #store the gui structure in something nice insted of python code
      menu = [('File',[('Exit',self.cleanup)]),
              ('Simulators',[['Load...',self.loadSim]]),
              ('Robot',[['Load...',self.loadRobot],
                        ['Unload',self.freeRobot]]),
              ('Brain',[['Load...',self.loadBrain],
                        ['Unload',self.freeBrain],
                        ['View', self.viewBrain]]),
              ('Plot',[['Load...',self.loadPlot]]),
              # ['robot', self.viewRobot]
              ('Move', [['Forward',self.stepForward],
                        ['Back',self.stepBack],
                        ['Left',self.stepLeft],
                        ['Right',self.stepRight],
                        ['Stop',self.stopEngine]]),
              ('Help',[['About',system.usage]])
              ]

      button1 = [('Step',self.stepEngine),
                 ('Reload',self.resetEngine),
                 ('Refresh',self.refresh),
                 ('Run',self.runEngine),
                 ('Edit', self.editBrain),
                 ('Stop',self.stopEngine)]

      # create menu
      self.mBar = Frame(self.frame, relief=RAISED, borderwidth=2)
      self.mBar.pack(fill=X)

      for entry in menu:
         self.mBar.tk_menuBar(self.makeMenu(entry[0],entry[1]))

      # if show main buttons:
      if 1: # FIX: add a preference about showing buttons someday
         toolbar = Frame(self.frame)
         toolbar.pack(side=TOP, fill=X)
         for b in button1:
            Button(toolbar,text=b[0],width=6,command=b[1]).pack(side=LEFT,padx=2,pady=2)

      if self.graphicsMode == 1: # GL
         self.win = Opengl(master = self.frame, width = width, \
                           height = height, double = db, depth = depth)
         self.win.pack(side = 'top', expand = 1, fill = 'both')
         self.win.winfo_toplevel().title("pyro@%s" % os.getenv('HOSTNAME'))
         self.win.redraw = self.redraw
         self.mode = IntVar(self.win)
         self.mode.set(GL_EXP)
      else: # TK, no display
         self.frame.winfo_toplevel().title("pyro@%s" % os.getenv('HOSTNAME'))
         self.win = self.frame

      self.commandFrame = Frame(self.frame)
      self.commandFrame['relief'] = 'raised'
      self.commandFrame['bd']	 = '2'
      self.commandFrame.pack({'expand':'no', 'side':'bottom', 'fill':'x'})

      self.commandLabel = Label(self.commandFrame)
      self.commandLabel["text"] = "Command:"
      self.commandLabel.pack({'expand':'no', 'side':'left', 'fill':'none'})
      # create a command 
      self.commandEntry = Entry(self.commandFrame)
      self.commandEntry.bind('<Return>', self.CommandReturnKey)
      self.commandEntry["relief"] = "ridge"
      self.commandEntry.pack({'expand':'yes', 'side':'right', 'fill':'x'})

      # create a status bar
      self.status = StatusBar(self.frame)
      self.status.pack(side=BOTTOM, fill=X)
      self.inform("Pyro Version " + version() + ": Ready...")

   def CommandReturnKey(self, event):
      from string import strip
      command = strip(self.commandEntry.get())
      self.commandEntry.delete(0, 'end')
      self.processCommand(command)
      #self.commandEntry.insert(0, filter)
      #self.commandButton.flash()
      #self.UpdateListBoxes()

   def viewRobot(self):
      self.windowRobot = Tk()
      self.windowRobot.wm_title("pyro@%s: Robot View" % os.getenv('HOSTNAME'))
      self.canvasRobot=Canvas(self.windowRobot,width=400,height=400)
      self.canvasRobot.pack()

   def viewBrain(self):
      self.windowBrain = Tk()
      self.windowBrain.wm_title("pyro@%s: Brain View" % os.getenv('HOSTNAME'))
      self.canvasBrain = Canvas(self.windowBrain,width=500,height=300)
      self.canvasBrain.pack()

   def redrawPie(self, pie, percentSoFar, piececnt, controller, percent, name):
      # FIX: behavior specific. How to put in Behavior-based code?
      xoffset = 5
      yoffset = 20
      width = 100
      row = (pie - 1) * (width * 1.5)
      colors = ['blue', 'red', 'tan', 'yellow', 'orange', 'black', 'azure', 'beige', 'brown', 'coral', 'gold', 'ivory', 'moccasin', 'navy', 'salmon', 'tan', 'ivory']
      try:
         self.canvasBrain.create_text(xoffset + 60,row + 10, tags='pie',fill='black', text = controller + ":")
         self.canvasBrain.create_arc(xoffset + 10,row + yoffset,width + xoffset + 10,row + width + yoffset,start = percentSoFar * 360.0, extent = percent * 360.0 - .001, tags='pie',fill=colors[(piececnt - 1) % 17])
         self.canvasBrain.create_text(xoffset + 275,row + 10 + piececnt * 20, tags='pie',fill=colors[(piececnt - 1) % 17], text = name)
      except:
         pass

   def redrawWindowBrain(self):
      # FIX: behavior specific. How to put this in behavior code so
      # that each brain would know how to draw itself?
      if self.windowBrain != 0:
         if self.engine and self.engine.brain and self.lastRun != self.engine.brain.lastRun:
            self.lastRun = self.engine.brain.lastRun
            self.canvasBrain.delete('pie')
            total = {}
            try:
               for c in self.engine.brain.effectsTotal.keys():
                  total[c] = self.engine.brain.effectsTotal[c]
            except:
               pass
            piecnt = 0
            keys = total.keys()
            keys.sort()
            for control in keys:
               piecnt += 1
               percentSoFar = 0
               piececnt = 0
               for d in self.engine.brain.desires:
                  if control == d[1] and total[d[1]] != 0:
                     piececnt += 1
                     portion = float(d[2])/float(total[d[1]]) * d[0]
                     self.redrawPie(piecnt, percentSoFar, \
                                    piececnt, \
                                    d[1] + " effects", \
                                    portion, d[4] + ":" + d[3] + ":" + d[6] + " IF %.2f THEN %.2f" % (d[0], d[5]))
                     percentSoFar += portion
         else:
            try:
               self.canvasBrain.create_text(200,130, tags='pie',fill='black', text = "Ready...")
            except:
               pass

   def redrawWindowRobot(self):
      #if self.windowRobot != 0:
      #   print dir(self.engine.robot),
      pass

   def editBrain(self):
      import os
      os.system("emacs " + self.engine.brainfile + "&")

   def makeMenu(self,name,commands):
      menu = Menubutton(self.mBar,text=name,underline=0)
      menu.pack(side=LEFT,padx="2m")
      menu.filemenu = Menu(menu)
      for cmd in commands:
         menu.filemenu.add_command(label=cmd[0],command=cmd[1])
      menu['menu'] = menu.filemenu
      return menu

   def run(self):
    self.win.mainloop()
    #while 1:
    #     self.redraw(self.win)
    #     self.win.tk.dooneevent()

   def init(self):
      # Do not specify a material property here.
      mat = [0.0, 0.0, 0.0, 1.0]
      glMaterialfv(GL_FRONT, GL_AMBIENT, mat)
      glMaterialfv(GL_FRONT, GL_DIFFUSE, [0.61424, 0.04136, 0.04136, 1.0])
      glMaterialfv(GL_FRONT, GL_SPECULAR, [0.727811, 0.626959, 0.626959, 1.0])
      glMaterialfv(GL_FRONT, GL_SHININESS, 0.6 * 128.0)
      lightgray = [0.75, 0.75, 0.75, 1.0]
      gray = [0.5, 0.5, 0.5, 1.0]
      darkgray = [0.25, 0.25, 0.25, 1.0]
      black = [0, 0, 0]
      
      glLightfv(GL_LIGHT0, GL_SPECULAR, lightgray);
      glLightfv(GL_LIGHT0, GL_DIFFUSE, gray);
      glLightfv(GL_LIGHT0, GL_AMBIENT, black);
      
      glDisable(GL_DITHER)
      glEnable(GL_DEPTH_TEST)
      glDepthFunc(GL_LESS)
      glLightfv(GL_LIGHT0, GL_POSITION, [0.0, 3.0, 3.0, 0.0])
      localview = [0.0]
      glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, localview )
      glEnable(GL_CULL_FACE)
      glEnable(GL_LIGHTING)
      glEnable(GL_LIGHT0)
      glEnable(GL_COLOR_MATERIAL)
      glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE)
      #self.win.tkRedraw()

   def fileloaddialog(self, filetype, skel):
      from string import replace
      import pyro
      from os import getcwd, getenv, chdir
      retval = ""
      cwd = getcwd()
      chdir(pyro.pyrodir() + "/plugins/" + filetype)
      d = LoadFileDialog(self.win, "Load " + filetype, skel)
      if d.Show() == 1:
         doc = d.GetFileName()
         d.DialogCleanup()
         retval = doc
      else:
         d.DialogCleanup()
      chdir(cwd)
      return retval

   def refresh(self):
      self.win.autospin = 1
      self.win.xspin = 0
      self.win.yspin = 0
      self.win.after(500, self.win.do_AutoSpin)

   def redraw(self, win = 0):
      glClearColor(0.5, 0.5, 0.5, 0)
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
      f = GenericStream()
      r = StreamRenderer(f)
      self.draw({}, r)
      f.close()
      s = StreamTranslator(f, GLRenderer())
      s.process()
      f.close()
      if self.genlist == 0:
         self.win.update()
         self.win.autospin = 1
         self.win.xspin = 0
         self.win.yspin = 0
         self.win.after(500, self.win.do_AutoSpin)
         self.genlist = 1
      # This probably doesn't need to update this often:
      self.redrawWindowBrain()
      self.redrawWindowRobot()
      for p in self.engine.plot:
         try:
            p.redraw(()) # pass in options
         except:
            pass # window closed, remove p?

if __name__ == '__main__':
   gui = GLgui()
   gui.inform("Ready...")
   gui.win.redraw = gui.redraw
   gui.init()
   gui.win.mainloop()
   gui.cleanup()
