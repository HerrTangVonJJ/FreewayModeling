import numpy as np

def divergeByProportion(totalFlow, prop):
    prop = np.array(prop)
    return totalFlow * prop / np.sum(prop)

def divergeByLogit(totalFow, nu, weights):
    weights = np.array(weights)
    return totalFow * np.exp(-nu*weights) / np.sum(np.exp(-nu*weights))

def mergeByPriority(subflow, maxFlow, priority):
    # Smaller number indicates higher priority.
    indices = sorted(range(len(priority)), key=lambda k: priority[k])
    routedFlow = np.zeros(len(priority))
    for i in indices:
        routedFlow[i] = min(maxFlow, subflow[i])
        maxFlow = max(maxFlow-subflow[i], 0)
    return routedFlow

def mergeByProportion(maxFlow, prop):
    prop = np.array(prop)
    return maxFlow * prop / np.sum(prop)


def getStatesFromCTMP(currentState, ctmp, timeStep):
    '''
    Get states from continuous-time markov process
    '''

    # transition is a dictionary, namely {state0: p0, state1: p1, ... } 
    transition = ctmp[currentState]
    states = list(transition.keys())

    prob = [transition[s]*timeStep for s in states]
    prob[states.index(currentState)] = 0
    prob[states.index(currentState)] = 1 - np.sum(prob)

    return np.random.choice(states, p=prob)

