import numpy as np
import Lib.utils as utils 

class MetaCell:
    def __init__(self, name, maxSendingFlow=None, maxReceivingFlow=None, cellLen=None):
        self.name = name
        self.cellLen = cellLen # km
        
        if maxSendingFlow is not None:
            self.maxSendingFlow = maxSendingFlow # veh/hour
        
        if maxReceivingFlow is not None:
            self.maxReceivingFlow = maxReceivingFlow # veh/hour
      
        self.state = {'numVeh': 0, 
                      'subInVeh': [], 
                      'subOutVeh': [], 
                      'inVeh': 0, 
                      'outVeh': 0}
        
        self.divergence = None
        self.merger = None

    def setPredecessor(self, predecessor):
        self.predecessor = predecessor
    
    def setSuccessor(self, successor):
        self.successor = successor
        
    def setDiverging(self, method, params):
        self.divergence = {'method': method, 'params': params}
    
    def setMerging(self, method, params):
        self.merger = {'method': method, 'params': params}

    def updateNumVeh(self):
        self.state['inVeh'] = np.sum(self.state['subInVeh'])
        self.state['outVeh'] = np.sum(self.state['subOutVeh'])
        self.state['numVeh'] += self.state['inVeh'] - self.state['outVeh']

    def addState(self, newState):
        for k, v in newState.items():
            self.state[k] = v

    def initializeSaver(self, savedState):
        self.saver = {}
        for k in savedState:
            self.saver[k] = []

    def saveData(self):
        for k in self.saver.keys():
            try:
                self.saver[k].append(self.state[k].copy())
            except:
                self.saver[k].append(self.state[k])

    def updateOutVeh(self, timeStep, stepSize):
        pass

    def updateInVeh(self, timeStep, stepSize):
        pass

class Source(MetaCell):
    def __init__(self, name, maxSendingFlow):
        super(Source, self).__init__(name, maxSendingFlow=maxSendingFlow)     
        self.addState({'maxOutVeh': 0})

    def setDemand(self, demand):
        self.demand = demand

    def updateOutVeh(self, timeStep, stepSize):
        # unit of stepSize: minute
        self.state['maxOutVeh'] =  min(self.demand[timeStep]*stepSize/60+self.state['numVeh'], self.maxSendingFlow*stepSize/60)
        
        if self.divergence is None:
            self.state['subOutVeh'] = [self.state['maxOutVeh']]
        elif self.divergence['method'] == 'proportion-based':
            prop = self.divergence['params']['proportion'][timeStep]
            self.state['subOutVeh'] = utils.divergeByProportion(self.state['maxOutVeh'], prop)
        else:
            print('Error in diverge!')

    def updateInVeh(self, timeStep, stepSize):
        self.state['subInVeh'] = [self.demand[timeStep]*stepSize/60]

class Sink(MetaCell):
    def __init__(self, name, maxReceivingFlow):
        super(Sink, self).__init__(name, maxReceivingFlow=maxReceivingFlow)
        self.addState({'maxInVeh': 0})

    def updateOutVeh(self, timeStep, stepSize):
        self.state['subOutVeh'] = [0]

    def updateInVeh(self, timeStep, stepSize):
        self.state['maxInVeh'] = self.maxReceivingFlow * stepSize/60

        # Number of vehicles from each upstream cell in one time step.
        subInVeh = [p.state['subOutVeh'][p.successor.index(self)] for p in self.predecessor]

        if np.sum(subInVeh) > self.state['maxInVeh']:
            # Clip subInVeh.
            if self.merger is None:
                self.state['subInVeh'] = [min(subInVeh[0], self.state['maxInVeh'])]
            elif self.merger['method'] == 'priority-based':
                self.state['subInVeh'] = utils.mergeByPriority(subInVeh, self.state['maxInVeh'], self.merger['params']['priority'])
            elif self.merger['method'] == 'proportion-based':
                if len(self.merger['params']['proportion']) == 0:
                    self.state['subInVeh'] = utils.mergeByProportion(self.state['maxInVeh'], subInVeh)
                else:
                    self.state['subInVeh'] = utils.mergeByProportion(self.state['maxInVeh'], self.merger['params']['proportion'])
            else:
                print('Error in merge!')
                
            # Broadcast clipped subInVeh.
            for i, p in enumerate(self.predecessor):
                clippedValue = min(self.state['subInVeh'][i], p.state['subOutVeh'][p.successor.index(self)])
                p.state['subOutVeh'][p.successor.index(self)] = clippedValue
                self.state['subInVeh'][i] = clippedValue
        else:
            self.state['subInVeh'] = subInVeh

class Cell(MetaCell):
    def __init__(self, name, freeFlowSpd, criticalDen, maxSendingFlow, congWaveSpd, maxReceivingFlow, length, droppedReceivingFlow=None):
        super(Cell, self).__init__(name, maxSendingFlow, maxReceivingFlow, length)
        self.addState({'den': 0,
                       'maxOutVeh': 0, 
                       'maxInVeh': 0})

        self.freeFlowSpd = freeFlowSpd # km/h
        self.congWaveSpd = congWaveSpd # km/h
        self.criticalDen = criticalDen # veh/km
        self.droppedReceivingFlow = droppedReceivingFlow # veh/h

        #self.capFluctuation = None
    
    #def setCapFluctuation(self, mode, params):
    #    self.capFluctuation = {}
    #    self.capFluctuation['mode'] = mode
    #    for k, v in params.items():
    #        self.capFluctuation[k] = v
 
    #def updateCap(self, timeStep):
    #    if self.capFluctuation['mode'] == 'markov process':
    #        # Set next state based on continuous-time markov process
    #        self.state['cap'] = utils.getStatesFromCTMP(self.state['cap'], self.capFluctuation['transition'])
    #    elif self.capFluctuation['mode'] == 'piecewise determinisitc':
    #        self.state['cap'] = self.capFluctuation['timeVaryingCap'][timeStep]
    
    def updateOutVeh(self, timeStep, stepSize):
        # Total number of vehicles allowed to leave in one time step.
        self.state['maxOutVeh'] = min(self.state['numVeh'], 
                                      self.maxSendingFlow*stepSize/60,
                                      self.freeFlowSpd*self.state['den']*stepSize/60)

        # Number of vehicles towards each downstream cell in one time step.
        if self.divergence is None:
            self.state['subOutVeh'] = [self.state['maxOutVeh']]
        elif self.divergence['method'] == 'proportion-based':
            prop = self.divergence['params']['proportion'][timeStep]
            self.state['subOutVeh'] = utils.divergeByProportion(self.state['maxOutVeh'], prop)
        else:
            print('Error in diverge!')

    def updateInVeh(self, timeStep, stepSize):
        # Total number of vehicles allowed to enter in one time step.
        if self.state['den'] > self.criticalDen:
            if self.droppedReceivingFlow is None:
                # No capacity drop.
                self.state['maxInVeh'] = max(self.maxReceivingFlow-self.congWaveSpd*(self.state['den']-self.criticalDen), 0)*stepSize/60
            else:
                # Capacity drop.
                self.state['maxInVeh'] = max(self.droppedReceivingFlow-self.congWaveSpd*(self.state['den']-self.criticalDen), 0)*stepSize/60
        else:
            # Uncongested.
            self.state['maxInVeh'] = self.maxReceivingFlow*stepSize/60

        # Number of vehicles from each upstream cell in one time step.
        subInVeh = [p.state['subOutVeh'][p.successor.index(self)] for p in self.predecessor]

        if np.sum(subInVeh) > self.state['maxInVeh']:
            # Clip subInVeh.
            if self.merger is None:
                self.state['subInVeh'] = [min(subInVeh[0], self.state['maxInVeh'])]
            elif self.merger['method'] == 'priority-based':
                self.state['subInVeh'] = utils.mergeByPriority(subInVeh, self.state['maxInVeh'], self.merger['params']['priority'])
            elif self.merger['method'] == 'proportion-based':
                if len(self.merger['params']['proportion']) == 0:
                    self.state['subInVeh'] = utils.mergeByProportion(self.state['maxInVeh'], subInVeh)
                else:
                    self.state['subInVeh'] = utils.mergeByProportion(self.state['maxInVeh'], self.merger['params']['proportion'])
            else:
                print('Error in merige!')
                
            # Broadcast clipped subInVeh.
            for i, p in enumerate(self.predecessor):
                clippedValue = min(self.state['subInVeh'][i], p.state['subOutVeh'][p.successor.index(self)])
                p.state['subOutVeh'][p.successor.index(self)] = clippedValue
                self.state['subInVeh'][i] = clippedValue
        else:
            self.state['subInVeh'] = subInVeh

    def updateNumVeh(self):
        self.state['inVeh'] = np.sum(self.state['subInVeh'])
        self.state['outVeh'] = np.sum(self.state['subOutVeh'])
        self.state['numVeh'] += self.state['inVeh'] - self.state['outVeh']
        self.state['den'] = self.state['numVeh'] / self.cellLen

class Network:
    def __init__(self, cellParams):
        # Initialize cells.
        self.initializeCells(cellParams)

        # Set neighbors.
        self.setNeighbors(cellParams)

        # Set demand
        self.setDemands(cellParams)

        
    def initializeCells(self, cellParams):
        self.cells = {}
        for name, cellParam in cellParams.items():
            if cellParam['type'] == 'cell':
                self.cells[name] = Cell(**cellParam['params'])
            elif cellParam['type'] == 'source':
                self.cells[name] = Source(**cellParam['params'])
            elif cellParam['type'] == 'sink':
                self.cells[name] = Sink(**cellParam['params'])
            else:
                print('Error in initializing cells!')

    def setNeighbors(self, cellParams):
        # Set neighbors for each cell
        for name, cellParam in cellParams.items():
            # Set predecessor
            if cellParam['neighbors']['predecessor'] is not None:
                predecessor = []
                for pName in cellParam['neighbors']['predecessor']:
                    predecessor.append(self.cells[pName])
                self.cells[name].setPredecessor(predecessor)

                if len(predecessor) > 1:
                    self.cells[name].setMerging(**cellParam['merging'])
            
            # Set successor
            if cellParam['neighbors']['successor'] is not None:
                successor = []
                for sName in cellParam['neighbors']['successor']:
                    successor.append(self.cells[sName])
                self.cells[name].setSuccessor(successor)

                if len(successor) > 1:
                    self.cells[name].setDiverging(**cellParam['diverging'])
    
    def setDemands(self, cellParams):
        for name, cellParam in cellParams.items():
            if type(self.cells[name]) == Source:
                self.cells[name].setDemand(cellParam['demand'])

    def updateNumVeh(self):
        [cell.updateNumVeh() for _, cell in self.cells.items()]

    def updateOutVeh(self, timeStep, stepSize):
        [cell.updateOutVeh(timeStep, stepSize) for _, cell in self.cells.items()]

    def updateInVeh(self, timeStep, stepSize):
        [cell.updateInVeh(timeStep, stepSize) for _, cell in self.cells.items()]

    def initializeSaver(self):
        [cell.initializeSaver(list(cell.state.keys())) for _, cell in self.cells.items()]

    def saveData(self):
        [cell.saveData() for _, cell in self.cells.items()]

class Simulation:
    def __init__(self, totalStep):
        self.totalStep = totalStep
    
    def initializeNetwork(self, cellParams):
        self.network = Network(cellParams)
    
    def initializeSaver(self):
        self.network.initializeSaver()

    def simulate(self, stepSize, isSaved=False):
        if isSaved:
            self.initializeSaver()

        for timeStep in range(self.totalStep):
            self.network.updateOutVeh(timeStep, stepSize)
            self.network.updateInVeh(timeStep, stepSize)
            self.network.updateNumVeh()
            if isSaved:
                self.network.saveData()
    
    def simulateOneStep(self, timeStep, stepSize, isSaved=False):
        self.network.updateOutVeh(timeStep, stepSize)
        self.network.updateInVeh(timeStep, stepSize)
        self.network.updateNumVeh()
        if isSaved:
            self.network.saveData()    