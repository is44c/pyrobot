from Tkinter import *

class Joystick:

   def __init__(self):
      self.app = Tk()
      self.app.withdraw()
      self.win = Toplevel()
      self.win.wm_title('Joystick')

      self.frame = Frame(self.win)
      self.canvas = Canvas(self.frame,
                           width = 220,
                           height = 220,
                           bg = 'white')
      
      self.canvas.bind("<ButtonRelease-1>", self.canvas_clicked_up)
      self.canvas.bind("<Button-1>", self.canvas_clicked_down)
      self.canvas.bind("<B1-Motion>", self.canvas_moved)
      self.canvas.pack(side=BOTTOM)

      self.circle_dim = (10, 10, 210, 210) #x0, y0, x1, y1
      self.circle = self.canvas.create_oval(self.circle_dim, fill = 'white')
      self.canvas.create_oval(105, 105, 115, 115, fill='black')

#      self.panicButton = Button(self.frame, text="STOP!!", command=self.stop)
#      self.panicButton.pack(side = TOP)
      self.frame.pack()
      self.translate = 0
      self.rotate = 0

   def move(self, translate, rotate):
      self.translate = translate
      self.rotate = rotate

   def destroy(self):
      self.win.destroy()

   def canvas_clicked_up(self, event):
      self.move(0, 0)

   def canvas_clicked_down(self, event):
      if self.in_circle(event.x, event.y):
         trans, rotate = self.calc_tr(event.x, event.y)
         self.move(trans, rotate)

   def canvas_moved(self, event):
      if self.in_circle(event.x, event.y):
         trans, rotate = self.calc_tr(event.x, event.y)
         self.move(trans, rotate)

   def stop(self):
      self.move(0, 0)

   def in_circle(self, x, y):
      r2 = ((self.circle_dim[2] - self.circle_dim[0])/2)**2
           
      center = ((self.circle_dim[2] + self.circle_dim[0])/2,
                (self.circle_dim[3] + self.circle_dim[1])/2)
      #x in?
      dist2 = (center[0] - x)**2 + (center[1] - y)**2
      if (dist2 < r2):
         return 1
      else:
         return 0

   def calc_tr(self, x, y):
      #right is negative
      center = ((self.circle_dim[2] + self.circle_dim[0])/2,
                (self.circle_dim[3] + self.circle_dim[1])/2)
      rot = float(center[0] - x) / float(center[0] - self.circle_dim[0])
      trans = float(center[1] - y) / float(center[1] - self.circle_dim[1])
      return (trans, rot)