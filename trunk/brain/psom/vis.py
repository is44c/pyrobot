from pyro.brain.psom import *
from pyro.brain.psom.visvector import *
from Tkinter import *

ACT_MAX   = 5
GRAY_STEP = 20

class VisPsom(psom):
   """
   A vizualized psom class.
   Takes all the same arguments as the psom class, with the following added
   keyword arguments:

   vis_radius:  The radius (in pixels) of the som cells.  Defaults to 15

   vis_padding: The amount of space (in pixels) on each side of each cell.
      Defaults to 2

   vis_vectortype:  The type of VisVector vizualier to use to display the
      model vectors.  Defaults to 'generic'.  See visvector.py.
   """

   def __init__(self, *args, **keys):
      self.last_x = 0
      self.last_y = 0
      #get Vis-specific keyword arguements out
      if 'vis_radius' in keys.keys():
         self.vis_radius = keys['vis_radius']
         del keys['vis_radius']
      else:
         self.vis_radius = 15
      if 'vis_padding' in keys.keys():
         self.vis_padding = keys['vis_padding']
         del keys['vis_padding']
      else:
         self.vis_padding = 2
      if 'vis_vectortype' in keys.keys():
         self.vectortype = keys['vis_vectortype']
         del keys['vis_vectortype']
      else:
         self.vectortype = "Generic"
      if 'title' in keys.keys():
         title = keys['title']
         del keys['title']
      else:
         title = "VisPsom"
      if 'opts' in keys.keys():
         self.opts = keys['opts']
         del keys['opts']
      else:
         self.opts = None

        
      psom.__init__(self, *args, **keys)
      self.app = Tk()
      self.app.wm_state('withdrawn')
      self.win = Toplevel()
      self.win.wm_title(title)

      cellwidth = (self.vis_padding + self.vis_radius) * 2
      #offset to set off the rows for a hexagonal topology
      if self.topol == 'hexa':
         offset = cellwidth/2
      else: #topol = 'rect'
         offset = 0
      self.canvas = Canvas(self.win,
                           width=self.xdim*(2*self.vis_radius+2*self.vis_padding) + offset,
                           height=self.ydim*(2*self.vis_radius+2*self.vis_padding),
                           bg='white')
      self.canvas.bind("<ButtonRelease-1>", self.canvas_clicked_up)
      self.canvas.bind("<Button-1>", self.canvas_clicked_down)
      self.canvas.pack(side=TOP)

      """
      # Toggle Button stuff (DEPRECATED)
      f = Frame(self.win)
      self.showCount = "Train"
      self.toggleCount = Button(f, text="Show Map Count",
                                command=self.countSwitch)
      self.toggleCount.pack()
      f.pack(side=BOTTOM)
      """
      
      self.lastMapped = (0,0)
      self.cells = []
      self.cellhash = {}
      self.history = {}

      # draw connection lines first:
      for y in range(self.ydim):
         self.cells.append([])
         for x in range(self.xdim):
            x0 = (cellwidth * x) + self.vis_padding + (offset * (y % 2))
            y0 = (cellwidth * y) + self.vis_padding
            x1 = x0 + (self.vis_radius * 2)
            y1 = y0 + (self.vis_radius * 2)
            #    1    2
            #     \  /
            #   0 -  - 3
            #     /  \
            #    5    4
            connection = [1] * 6
            if y == 0:               # top row
               connection[1] = 0; connection[2] = 0
            if x == 0:               # left row
               connection[0] = 0;
            if y % 2 == 0 and x == 0:
               connection[1] = 0
            if y % 2 == 1 and x == self.xdim - 1:
               connection[2] = 0
            if connection[0]: self.canvas.create_line(x0 + cellwidth / 2,
                                                      y0 + cellwidth / 2,
                                                      x0 - cellwidth / 2,
                                                      y0 + cellwidth / 2,
                                                      tags = 'cell')
            if connection[1]: self.canvas.create_line(x0 + cellwidth / 2,
                                                      y0 + cellwidth / 2,
                                                      x0,
                                                      y0 - cellwidth / 2,
                                                      tags = 'cell')
            if connection[2]: self.canvas.create_line(x0 + cellwidth / 2,
                                                      y0 + cellwidth / 2,
                                                      x0 + cellwidth,
                                                      y0 - cellwidth / 2,
                                                      tags = 'cell')

      for y in range(self.ydim):
         self.cells.append([])
         for x in range(self.xdim):
            x0 = (cellwidth * x) + self.vis_padding + (offset * (y % 2))
            y0 = (cellwidth * y) + self.vis_padding
            x1 = x0 + (self.vis_radius * 2)
            y1 = y0 + (self.vis_radius * 2)
            cell = self.canvas.create_oval(x0, y0, x1, y1, fill='white',
                                           tags = 'cell')
            center = ((x0 + x1)/2, (y0 + y1)/2)
            
            # display settings for train count
            traintext = self.canvas.create_text(center[0], center[1],
                                                text = "",
                                                fill = 'red',
                                                tags = 'traincount')
            # display settings for map count
            maptext = self.canvas.create_text(center[0], center[1],
                                              text = "",
                                              fill = 'blue',
                                              tags = 'mapcount')
            # display settings for label
            labeltext = self.canvas.create_text(center[0], center[1],
                                                text = "",
                                                fill = 'purple',
                                                tags = 'label')
            # dictionary associated with each cell
            pt = point(x, y)
            self.cells[y].append({"cell": cell,
                                  "traincount": self.get_counter(pt, 'train'),
                                  "mapcount": self.get_counter(pt, 'map'),
                                  "traintext": traintext,
                                  "maptext": maptext,
                                  "labeltext": labeltext,
                                  "label": self.get_model_vector(pt).get_label()})
            self.cellhash[cell] = (x, y)

      self.canvas.tag_lower('cell', 'traincount')
      self.canvas.tag_lower('mapcount', 'cell')
      self.canvas.tag_lower('label', 'cell')

      # Set labels
      for y in range(self.ydim):
         for x in range(self.xdim):
            self._labelcell(x, y)

      # menu bar with File and Show options
      menuBar = Menu(self.win)
      self.win.config(menu=menuBar)
      FileBtn = Menu(menuBar)
      menuBar.add_cascade(label='File', menu=FileBtn)
      FileBtn.add_command(label='Exit', command=self.close)

      ShowBtn = Menu(menuBar)
      menuBar.add_cascade(label='Show', menu=ShowBtn)
      ShowBtn.add_radiobutton(label='Train Count',
                              command=self.show_train_count)
      ShowBtn.add_radiobutton(label='Map Count',
                              command=self.show_map_count)
      ShowBtn.add_radiobutton(label='Labels',
                              command=self.show_labels)
      ShowBtn.invoke(ShowBtn.index('Train Count')) # show train count by default
      # end menu bar

   def close(self):
      self.win.destroy()
      
   def canvas_clicked_up(self, event):
      celllist = self.canvas.find_overlapping(event.x, event.y,
                                              event.x, event.y)
      cell = None
      for item in celllist:
         if item in self.cellhash.keys():
            cell = item
            break
      
      label = "No Label"
      if cell:
         x, y = self.cellhash[cell]
         vec = self.get_model_vector(point(x, y))
         
         # build string of items in label ls
         label_ls = self.cells[y][x]['label']
         if len(label_ls) != 0:
            label = ""
            for item in label_ls:
               label += item
               
         if x == self.last_x and y == self.last_y:
            visclass = getVisVectorByName(self.vectortype)
            if self.opts: # override defaults
               visclass(vec, title="(%d,%d):%s" % (x, y, label),
                        opts = self.opts)
            else:
               visclass(vec, title="(%d,%d):%s" % (x, y, label))
         else:
            # show difference
            vec2 = self.get_model_vector(point(self.last_x, self.last_y))
            diffvec = []
            for v in range(len(vec2)):
               diffvec.append( vec[v] - vec2[v] )
            myvector = vector( diffvec )
            visclass = getVisVectorByName(self.vectortype)
            if self.opts:
               visclass(myvector, title="(%d,%d) diff (%d,%d)"
                        % (x, y, self.last_x, self.last_y), opts = self.opts)
            else:
               visclass(myvector, title="(%d,%d) diff (%d,%d)"
                        % (x, y, self.last_x, self.last_y))

   def canvas_clicked_down(self, event):
      celllist = self.canvas.find_overlapping(event.x, event.y,
                                              event.x, event.y)
      cell = None
      for item in celllist:
         if item in self.cellhash.keys():
            cell = item
            break

      if cell:
         self.last_x, self.last_y = self.cellhash[cell]

   """
   # callback associated with toggle button (DEPRECATED)
   def countSwitch(self):
      if self.showCount == "Map":
         self.canvas.tag_raise('traincount', 'cell')
         self.canvas.tag_lower('mapcount', 'cell')
         self.toggleCount.configure(text="Show Map Count")
         self.showCount = "Train"
      elif self.showCount == "Train":
         self.canvas.tag_raise('mapcount', 'cell')
         self.canvas.tag_lower('traincount', 'cell')
         self.toggleCount.configure(text="Show Train Count")
         self.showCount = "Map"
   """
         
   def _setcount(self, x, y, new_counter, which):
      """
      Update the hit counter of a cell, and change the corresponding
      counter label.
      """
      self.cells[y][x][which+"count"] = new_counter
      self.canvas.itemconfigure(self.cells[y][x][which+"text"],
                                text=str(new_counter))
                                
   def _setcell(self, x, y, level):
      try:
         self.canvas.itemconfigure(self.cells[y][x]["cell"],
                                   fill='gray' + str(level))
      except:
         pass

   def _updatefill(self):
      for pt in self.history.keys():
         act = self.history[pt]
         self._setcell(pt[0], pt[1], (ACT_MAX - act) * GRAY_STEP)
         #decrease activation by 1
         if act == 0:
            del self.history[pt]
         else:
            self.history[pt] -= 1

   def _labelcell(self, x, y):
      """
      Given x, y coordinates, this function labels the corresponding cell
      using information stored in the dictionary associated with the cell.
      """
      label_ls = self.cells[y][x]['label']
      label = ""
      if len(label_ls) != 0:
         for item in label_ls:
            label += item
      self.canvas.itemconfigure(self.cells[y][x]['labeltext'],
                                text = label,
                                font=(('MS', 'Sans', 'Serif'), '8'))

   def clearfill(self):
      """
      Clears the markers, the count printout, and resets the count to 0
      for all cells.
      """
      for y in range(self.ydim):
         for x in range(self.xdim):
            self._setcell(x, y, 100)
            self.cells[x][y]['count'] = 0
      self.canvas.delete('count')
   
   def map(self, vector):
      model = psom.map(self, vector)
      pt    = model.point.x, model.point.y

      # change the color of the last mapped cell to gray
      self._setcell(self.lastMapped[0], self.lastMapped[1], 100)

      # color the current winning cell green
      self.canvas.itemconfigure(self.cells[pt[1]][pt[0]]["cell"],
                                fill = 'green')
      self.lastMapped = pt
      self._setcount(pt[0], pt[1],
                     self.get_counter(model.point, 'map'),
                     'map')
      self._labelcell(pt[0], pt[1])
      self.update()
      return model

   def train(self, vector):
      model = psom.train(self,vector)
      pt   = model.point.x, model.point.y
      
      self.history[pt] = ACT_MAX # max color activation (black)
      self._updatefill() # update color activations of nodes
      
      self._setcount(pt[0], pt[1],
                     self.get_counter(model.point, 'train'),
                     'train')
      self.update()
      return model

   def add_label(self, x, y, label=[]):
      """
      Given a label list, this function adds the label to the cell/model vector
      at the specified x,y position.  Previous label associations are preserved. 
      """
      vec = self.get_model_vector(point(x, y))
      vec.add_label(label)
      self.cells[y][x]['label'] = self.get_model_vector(point(x, y)).get_label()
      self._labelcell(x, y)

   def clear_label(self, x, y):
      """
      Removes all labels associated with the cell/model vector at the given
      x,y position.
      """
      vec = self.get_model_vector(point(x, y))
      vec.clear_label()
      self.cells[y][x]['label'] = self.get_model_vector(point(x, y)).get_label()
      self._labelcell(x, y)
      
   def update(self):
      while self.win.tk.dooneevent(2): pass

   def show_train_count(self):
      self.canvas.tag_raise('traincount', 'cell')
      self.canvas.tag_lower('mapcount', 'cell')
      self.canvas.tag_lower('label', 'cell')
      self.canvas.update()

   def show_map_count(self):
      self.canvas.tag_raise('mapcount', 'cell')
      self.canvas.tag_lower('traincount', 'cell')
      self.canvas.tag_lower('label', 'cell')
      self.canvas.update()

   def show_labels(self):
      self.canvas.tag_raise('label', 'cell')
      self.canvas.tag_lower('traincount', 'cell')
      self.canvas.tag_lower('mapcount', 'cell')
      self.canvas.update()

   
if __name__ == "__main__":
   def pause():
      print "Press [Enter] to continue...",
      raw_input();
   #mysom = VisPsom(file='ex.cod', vis_vectortype="Hinton")
   mysom = VisPsom(file='ex.cod')
   mydataset = dataset(file='ex.dat')
   mysom.init_training(0.02,4.0,5002)
   print "---> Begin training from dataset..."
   mysom.timing_start()
   mysom.train_from_dataset(mydataset)
   mysom.timing_stop()
   ttime = mysom.get_training_time()
   print "---> 5000 Training steps complete: %s seconds" % ttime
   pause()
   print "---> Training..."
   mysom.train(vector([30.0, 20.0, -.3, -.8, 400.0]))
   pause()
   print "---> Training..."
   mysom.train(vector([14.0, 10.0, .3, -.8, 400.0]))
   pause()
   print "---> Mapping..."
   mysom.map(vector([10.0, 30.0, -.3, .8, 375.0]))
   pause()
   print "---> Mapping..."
   mysom.map(vector([30.0, 20.0, -.3, -.8, 400.0]))
   pause()
   print "---> Mapping..."
   mysom.map(vector([30.0, 20.0, -.3, -.8, 400.0]))
   pause()
   print "---> Mapping..."
   mysom.map(vector([30.0, 20.0, -.3, -.8, 400.0]))
   pause()
   print "---> Mapping..."
   mysom.map(vector([30.0, 20.0, -.3, -.8, 400.0]))
   pause()
   print "---> Mapping..."
   mysom.map(vector([30.0, 20.0, -.3, -.8, 400.0]))
   pause()
   print "---> Mapping..."
   mysom.map(vector([30.0, 20.0, -.3, -.8, 400.0]))

   print "---> Adding 1 to the label at 0,0..."
   pause()
   mysom.add_label(0, 0, [1])

   print "---> Adding 'zc' to the label at 5,5..."
   pause()
   mysom.add_label(5, 5, ['zc'])

   print "---> Clearing label '' at 0,1..."
   pause()
   mysom.clear_label(0, 1)

   print "---> Clearing label 'B' at 1,0..."
   pause()
   mysom.clear_label(1, 0)

   print "---> Displaying dataset"
   pause()
   mysom.display()

   print "---> DONE. Please close window."
   mysom.win.mainloop()
