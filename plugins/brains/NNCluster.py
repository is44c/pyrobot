# A Neural Network Brain
# D.S. Blank

from pyro.brain import Brain
from pyro.brain.conx import *
from pyro.gui.plot.scatter import *
from pyro.gui.plot.hinton import *
from os import system, unlink

def a2s(a):
   """ array to string """
   retval = ''
   for i in range(len(a)):
      retval += "%.5f " % a[i]
   return retval

class NNBrain(Brain):
   """
   This is an example brain controlled by a neural network.
   This simple example loads the range sensor data into the
   input layer, and trains the network to stay away from
   things.
   """
   def setup(self):
      """ Ceate the network. """
      self.net = Network()
      self.hiddenLayerSize = 10
      self.net.addThreeLayers(self.getRobot().get('range', 'count'),
                              self.hiddenLayerSize, 2)
      self.net.setBatch(0)
      self.net.initialize()
      self.net.setEpsilon(0.5)
      self.net.setMomentum(.1)
      self.net.setLearning(1)
      self.counter = 0
      self.maxvalue = self.getRobot().get('range', 'maxvalue')
      self.hidScat = Scatter(title = 'Hidden Layer Activations',
                             history = [100, 2, 2], linecount = 3,
                             legend=['Hidden', 'Motor Out', 'Motor Target'])
      self.hidHinton = Hinton(self.hiddenLayerSize, title = 'Hidden Layer')
      self.inHinton = Hinton(self.getRobot().get('range', 'count'),
                             title = 'Input Layer')
      self.outHinton = Hinton(2, title = 'Output Layer')

   def destroy(self):
      self.hidScat.destroy()
      self.hidHinton.destroy()
      self.inHinton.destroy()
      self.outHinton.destroy()

   def scale(self, val):
      return (val / self.maxvalue)
   
   def step(self):
      if self.counter < 500:
         mode = 'learn'
      elif self.counter == 500:
         # stop learning, open file
         mode = 'open file'
         self.net.setLearning(0)
         # this will create a log file, and write the activations out
         # on each propagate:
         self.net.getLayer('hidden').setLog("hidden.dat")
      elif self.counter < 1000:
         mode = 'collect'
         # collect hid data when propagate
      elif self.counter == 1000:
         mode = 'close file'
         # close file, compute eigenvalues
         self.net.getLayer('hidden').closeLog()
         try:
            unlink("hidden.e")
         except:
            pass
         system("tools/cluster/cluster -pehidden.e -c1,2 hidden.dat > /dev/null")
      else:
         mode = 'plot'
         # plot data in PCA space
         pass 

      print self.counter
         
      # First, set inputs and targets:
      ins = map(self.scale, self.getRobot().get('range', 'value', 'all'))
      self.net.setInputs([ ins ])
      # Compute targets:
      if self.getRobot().get('range', 'value', 'front', 'min')[1] < 1:
         target_trans = 0.0
      elif self.getRobot().get('range', 'value', 'back', 'min')[1] < 1:
         target_trans = 1.0
      else:
         target_trans = 1.0
      if self.getRobot().get('range', 'value', 'left', 'min')[1] < 1:
         target_rotate = 0.0
      elif self.getRobot().get('range', 'value', 'right', 'min')[1] < 1:
         target_rotate = 1.0
      else:
         target_rotate = 0.5
      self.net.setOutputs([[target_trans, target_rotate]])
      # next, cycle through inputs/outputs:
      #print "Learning: trans =", target_trans, "rotate =", target_rotate
      self.net.sweep()
      # get PCA, components #1 and #2, then plot:
      if mode == 'plot':
         system("echo " + a2s(self.net.getLayer('hidden').activation) + " | tools/cluster/cluster -pehidden.e -c1,2 > out")
         plot = open("out").readline().split(' ')
         self.hidScat.addPoint( (float(plot[0]) + 1.0) / 2.0,
                                (float(plot[1]) + 1.0) / 2.0 )
      # get the output, and move:
      trans = (self.net.getLayer('output').activation[0] - .5) / 2.0
      rotate = (self.net.getLayer('output').activation[1] - .5) / 2.0
      self.hidScat.addPoint( trans * 2 + .5,
                             rotate * 2 + .5, 1)
      self.hidScat.addPoint( target_trans,
                             target_rotate, 2)
      self.inHinton.update(self.net.getLayer('input').activation)
      self.hidHinton.update(self.net.getLayer('hidden').activation)
      self.outHinton.update(self.net.getLayer('output').activation)
      #print "Move    : trans =", trans, "rotate =", rotate
      self.getRobot().move(trans, rotate)
      self.counter += 1

def INIT(engine):
   return NNBrain('NNBrain', engine)
      
