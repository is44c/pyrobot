"""

Governor code for self regulating networks.

"""

import pyro.brain.ravq
import random

class Governor:

    def __init__(self, size = 5, epsilon = 0.2, delta = 0.6, bufferSize = 50, mask = []):
        # ravq
        self.ravq = pyro.brain.ravq.RAVQ(size, epsilon, delta)
        self.ravq.setHistory(0)
        self.ravq.setAddModels(1)
        if not mask == []: 
            self.ravq.setMask(mask)

        # buffer 
        self.buffer = []
        self.bufferSize = bufferSize
        self.bufferIndex = 0

    def process(self, vector):
        self.ravq.input(vector)
        if self.ravq.getNewWinner(): 
            if len(self.buffer) >= self.bufferSize:
                self.buffer = self.buffer[1:] + [vector]
            else:
                self.buffer.append(vector)
        if len(self.buffer) > 0: 
            array = self.buffer[self.bufferIndex]
            self.bufferIndex = (self.bufferIndex + 1) % len(self.buffer)
            return array
        else:
            return vector

    def __len__(self):
        return len(self.ravq.models)

    def query(self):
        return str(self.ravq)
    
    def save(self, filename):
        self.ravq.saveRAVQToFile(filename)

    def load(self, filename):
        self.ravq.loadRAVQFromFile(filename)


class AdaptiveGovernor(Governor):

    def __init__(self, size = 5, epsilon = 0.2, delta = 0.6, alpha = 0.02, bufferSize = 50, mask = []):
        # ravq
        self.ravq = pyro.brain.ravq.ARAVQ(size, epsilon, delta, alpha)
        self.ravq.setHistory(0)
        self.ravq.setAddModels(1)
        if not mask == []: 
            self.ravq.setMask(mask)

        # buffer 
        self.buffer = []
        self.bufferSize = bufferSize
        self.bufferIndex = 0

class BalancingGovernor(Governor):

    def __init__(self, size = 5, epsilon = 0.2, delta = 0.6, alpha = 0.02, depth = 5, mask = []):
        # ravq
        self.ravq = pyro.brain.ravq.ARAVQ(size, epsilon, delta, alpha)
        self.ravq.setHistory(1)
        self.ravq.setHistorySize(depth)
        self.ravq.setAddModels(1)
        if not mask == []: 
            self.ravq.setMask(mask)
        self.index = -1

    def process(self, vector):
        self.ravq.input(vector)
        if self.ravq.getHistoryLength() is 0:
            return vector
        else:
            self.index = (1 + self.index) % self.ravq.getHistoryLength()
            return self.ravq.getHistory(self.index)

    # Definition borrowed from conx.py: I want to randomize the order
    # of training across the buffers periodically, but when to do so
    # is yet another parameter, so this is not yet implimented.
    
    def randomIndices(self, number):
        """
        random list generator
        """
        self.indices = [0] * number
        flag = [0] * number
        for i in range(number):
            pos = int(random.random() * number)
            while (flag[pos] == 1):
                pos = int(random.random() * number)
            flag[pos] = 1
            self.indices[pos] = i   

class DiscreteGovernor:

    def __init__(self, sampleDelay, bufferSize):
        self.delay = sampleDelay
        self.counter = 0

        # buffer 
        self.buffer = []
        self.bufferSize = bufferSize
        self.bufferIndex = 0

    def process(self, vector):
        if self.counter % self.delay == 0: 
            if len(self.buffer) >= self.bufferSize:
                self.buffer = self.buffer[1:] + [vector]
            else:
                self.buffer.append(vector)
        self.counter += 1
        if len(self.buffer) > 0: 
            array = self.buffer[self.bufferIndex]
            self.bufferIndex = (self.bufferIndex + 1) % len(self.buffer)
            return array
        else:
            return vector

    def query(self):
        pass
    
    def save(self, filename):
        pass

    def load(self, filename):
        pass
        

class RandomGovernor:

    # for random sampling
    def flip(self):
        """
        Flip a biased coin.
        """
        return random.random() <= self.probability


    def exponential(self):
        """
        Sample from discrete exponential distribution.
        """
        value = random.expovariate(1.0/10.0)
        return int(value)

    def waitTime(self):
        """
        Discrete distribution based on histogram data.
        """
        listp = [ 0.00595745,  0.0093617,   0.02553191,  0.04425532, 0.05787234,  0.08085106,
                  0.14382979,  0.2212766,   0.31829787, 0.39744681, 0.46893617,  0.53531915,
                  0.61531915,  0.70468085,  0.7787234 ,  0.85361702, 0.90468085,  0.92851064,
                  0.94808511,  0.95744681,  0.96255319,  0.96510638, 0.97361702,  0.98297872,
                  0.9906383 ,  0.99659574,  1.00]

        
        randomnumber = random.random()
        for i in range(len(listp)):
            if randomnumber < listp[i]:
                break
        return i

    def __init__(self, probability, bufferSize):
        self.probability = probability

        # buffer 
        self.buffer = []
        self.bufferSize = bufferSize
        self.bufferIndex = 0
        self.wait = self.waitTime()

    def process(self, vector):
        if self.wait is 0:
            self.wait = self.waitTime() # reset wait
            if len(self.buffer) >= self.bufferSize:
                self.buffer = self.buffer[1:] + [vector]
            else:
                self.buffer.append(vector)
        else:
            self.wait -= 1 # decrement wait
        # decide what to return
        if len(self.buffer) > 0: 
            array = self.buffer[self.bufferIndex]
            self.bufferIndex = (self.bufferIndex + 1) % len(self.buffer)
            return array
        else:
            return vector

    def query(self):
        pass
    
    def save(self, filename):
        pass

    def load(self, filename):
        pass
