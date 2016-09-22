__author__ = 'Abhishek'
from Queue import *
from sets import *
from customPriorityQueue import *

class PacmanState():
    parent = None
    pathCost = float('inf')
    def __init__(self, location=(-1,1), goalsLeft=((-1,1)), value='%'):
        self.location = location
        self.goalsLeft = tuple(goalsLeft)
        self.value = value

    def __hash__(self):
        return hash((self.location, self.goalsLeft))

    def __eq__(self, other):
        return (self.location, self.goalsLeft) == (other.location, other.goalsLeft)

    def markGoalComplete(self, goal):
        temp = []
        for g in self.goalsLeft:
            if g != goal:
                temp.append(g)
        self.goalsLeft = tuple(temp)

    def copy(self):
        newNode = PacmanState()
        newNode.parent = self.parent
        newNode.pathCost = self.pathCost
        newNode.location = self.location
        newNode.goalsLeft = self.goalsLeft
        newNode.value = self.value
        return newNode

class PacmanProblem():
    def __init__(self, grid, startState, goalStates, strategy, wallChar='%', goalChar='.', startChar='P', visualization=False):
        self.grid = grid
        self.gridHeight = len(grid)
        if self.gridHeight > 0:
            self.gridWidth = len(grid[0])
        else:
            self.gridWidth = 0

        self.startState = PacmanState()
        self.startState.location = startState
        self.startState.goalsLeft = tuple(goalStates)
        self.startState.pathCost = 0
        self.endState = PacmanState()

        self.goalStates = goalStates
        self.strategy = strategy
        self.wallChar='%'
        self.goalChar='.'
        self.startChar='P'
        self.outputName = ''

    def getNextPacmanStates(self, currState, wallChar='%', goalChar='.', startChar='P'):
        deltas = [1,-1]
        actionMap = {'l':[0,-1], 'r':[0,1], 'u':[-1,0], 'd':[1,0]}
        actions = actionMap.keys()
        states = []
        currY = currState.location[0]
        currX = currState.location[1]
        for action in actionMap:
            newY = currY + actionMap[action][0]
            newX = currX + actionMap[action][1]
            if self.grid[currY][currX].value == wallChar:
                actions = []
            elif newX < 0 or newY < 0 or newX > self.gridWidth or newY > self.gridHeight or self.grid[newY][newX].value == wallChar:
                actions.remove(action)
            else:
                states.append((newY, newX))
        return states

    def initializeFrontier(self):
        self.frontierDetails = {}
        if self.strategy == 'BFS':
            self.frontier = Queue()
        elif self.strategy == 'DFS':
            self.frontier = LifoQueue()
        elif self.strategy == 'greedy' or self.strategy == 'Astar':
            self.frontier = customPriorityQueue()
            self.frontier.pq = []    # WHY IS THIS NOT NULL?!?!?!

    def pushToFrontier(self, state):
        if self.strategy=='BFS' or self.strategy=='DFS':
            self.frontier.put(state)
        elif self.strategy=='greedy':
            h = self.getHeuristicValue(state)
            self.frontier.put(h, state)
        elif self.strategy=='Astar':
            h = self.getHeuristicValue(state)
            g = state.pathCost
            self.frontier.put(g+h, state)
        self.frontierDetails[state] = state

    def getFromFrontier(self):
        popped = self.frontier.get()
        self.frontierDetails.pop(popped)
        return popped

    def initializeExplored(self):
        self.explored = {}
        self.exploredOrderSequence = []

    def getHeuristicValue(self, currState):
        h = float('inf')
        for goal in self.goalStates:
            dist = abs(goal[0]-currState.location[0])+abs(goal[1]-currState.location[0])
            h = min(h, dist)
        return h

    def getSolutionPath(self):
        path = []
        curr = self.endState
        while curr!=None:
            path.append(curr)
            curr = self.explored[curr.parent]
            if curr == self.startState:
                break
        path.append(self.startState)
        path.reverse()
        return path

    def getPathAll(self):
        path = self.getSolutionPath()
        pathAll = []
        for element in path:
            pathAll.append((element.location, element.goalsLeft, element.pathCost))
        return pathAll

    def getPathCoordinates(self):
        path = self.getSolutionPath()
        pathLocs = []
        for element in path:
            pathLocs.append(element.location)
        return pathLocs