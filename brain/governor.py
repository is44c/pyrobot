"""

Governor code for self regulating networks.

"""

from pyro.brain.conx import *
from pyro.brain.ravq import ARAVQ, euclideanDistance

class Governor:
    """
    A Network + RAVQ. 
    """
    def incompatibility(self):
        """
        For each model, how different is it from each of the buffer items? Returns list of
        incompatibilities. Note: uses mask to scale.
        """
        retval = []
        for model in self.ravq.models:
            sum = 0.0
            for buff in model.contents:
                sum += euclideanDistance(model.vector, buff, self.ravq.mask)
            retval.append(sum)
        return retval

    def distancesTo(self, vector):
        """
        Computes euclidean distance from a vector to all model vectors. Note: uses mask
        to scale.
        """
        retval = []
        for m in self.ravq.models:
            retval.append( euclideanDistance(vector, m.vector, self.ravq.mask) )
        return retval

    def map(self, vector):
        """
        Returns the index and vector of winning position. Side effect: records
        index of winning pos in histogram.
        """
        index, modelVector = self.ravq.input(vector)
        if self.histogram.has_key(index):
            self.histogram[index] += 1
        else:
            if index >= 0:
                self.histogram[index] = 1
            # else ignore?
        return (index, modelVector)

    def nextItem(self):
        """ Public interface for getting next item from RAVQ. """
        retval = None
        try:
            retval = self.ravq.models.nextItem().nextItem()
        except AttributeError:
            pass
        return retval

    def __nextitem__(self):
        """ For use in iterable positions:
            >>> govnet = GovernorNetwork()
            >>> for item in govnet:
            ...    print item
            ...
        """
        return self.ravq.models.nextItem().nextItem()

    def saveRAVQ(self, filename):
        """ Saves RAVQ data to a file. """
        self.ravq.saveRAVQToFile(filename)

    def loadRAVQ(self, filename):
        """ Loads RAVQ data from a file. """
        self.ravq.loadRAVQFromFile(filename)

class GovernorNetwork(Governor, Network):
    def __init__(self, bufferSize = 5, epsilon = 0.2, delta = 0.6,
                 historySize = 5, alpha = 0.02, mask = [], verbosity = 0):
        # network:
        Network.__init__(self, name = "Governed Network",
                         verbosity = verbosity)
        # ravq
        self.governing = 1
        self.ravq = ARAVQ(bufferSize, epsilon, delta, historySize, alpha) 
        self.ravq.setAddModels(1)
        self.ravq.setVerbosity(verbosity)
        self.histogram = {}
        if not mask == []: 
            self.ravq.setMask(mask)

class GovernorSRN(Governor, SRN): 
    def __init__(self, bufferSize = 5, epsilon = 0.2, delta = 0.6,
                 historySize = 5, alpha = 0.02, mask = [], verbosity = 0):
        # network:
        SRN.__init__(self, name = "Governed Acting SRN",
                     verbosity = verbosity)
        self.trainingNetwork = SRN(name = "Governed Training SRN",
                                   verbosity = verbosity)
        # so that we can use step:
        self.trainingNetwork.initContext = 0
        self.initContext = 0 
        # ravq
        self.governing = 1
        self.ravq = ARAVQ(bufferSize, epsilon, delta, historySize, alpha) 
        self.ravq.setAddModels(1)
        self.ravq.setVerbosity(verbosity)
        self.histogram = {}
        if not mask == []: 
            self.ravq.setMask(mask)

    def addThreeLayers(self, i, h, o):
        SRN.addThreeLayers(self, i, h, o)
        self.trainingNetwork.addThreeLayers(i, h, o)
        self.shareWeights(self.trainingNetwork)

    def sweep(self):
        if self.epoch % self.reportRate == 0:
            print "MODEL VECTORS: ", len(self.ravq.models), self.histogram
            self.histogram = {}
        return SRN.sweep(self)

    def networkStep(self, **args):
        if self.governing:
            # map the ravq input context and target
            actContext = list(self["context"].activation)
            vector = args["input"] + actContext + args["output"]
            self.map(vector)
            # get the next
            inLen = self["input"].size
            conLen = self["context"].size
            outLen = self["output"].size
            array = self.nextItem()
            if array == None:
                array = vector
            input = array[0:inLen]
            context = array[inLen:inLen+conLen]
            output  = array[inLen+conLen:]
            # load them and train training Network
            self.trainingNetwork.step(input=input,
                                      output=output,
                                      context=context)
        return Network.step(self, **args)
        

if __name__ == '__main__':
    import os, gzip
    # read in experimental training data
    locationfile = gzip.open('location.dat.gz', 'r')
    sensorfile = gzip.open('sensors.dat.gz', 'r')
    sensors = sensorfile.readlines()
    locations = locationfile.readlines()
    locationfile.close()
    sensorfile.close()
    inputs = []
    for line in sensors:
        inputs.append( map(lambda x: float(x), line.strip().split()))
    targets = []
    for line in locations:
        targets.append( map(lambda x: float(x), line.strip().split()))
    # The "16" weights the input determining the multiple labels
    # The choice of epsilon and delta may change the required
    # weights. For binary nodes, changing the value will make the vector
    # distance one from every vector with the opposite value in that
    # node. This change is enough if the delta value is less than
    # one. Use of a high weight value is more to reflect that that
    # node is important in determining the function of the network.
    # create network
    inSize = len(sensors[0].strip().split())
    govMask = [1] * (inSize - 1) + [16] + [inSize/(inSize/2)] * (inSize/2) + [inSize/4] * 4
    net = GovernorSRN(5, 2.1, 0.3, 5, 0.2)
    net.addThreeLayers(inSize, inSize/2, 4)
    net.setInputs( inputs )
    net.setTargets( targets )
    net.setReportRate(1)
    net.governing = 1
    #net.setVerbosity(1)
    print "This takes a while..."
    net.train()
print net.ravq
net.setLearning(0)
net.setInteractive(1)
# run with -i to see net

