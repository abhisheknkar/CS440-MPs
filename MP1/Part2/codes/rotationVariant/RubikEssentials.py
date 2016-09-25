__author__ = 'Abhishek'
import numpy as np
import operator
from Queue import *
from customPriorityQueue import *

class RubikState:
    goalState = ('r', 'r', 'r', 'r', 'p', 'p', 'p', 'p', 'b', 'b', 'b', 'b', 'o', 'o', 'o', 'o', 'g', 'g', 'g', 'g', 'y', 'y', 'y', 'y')
    parentState = -1
    parentMove = None
    pathCost = float('inf')
    def __init__(self, filePath=None):
        '''
        +-------+-------+-------+
        | 0  1  | 8   9 | 16 17 |
        | 3  2  | 11 10 | 19 18 |
        +-------+-------+--------+
                | 12 13 |
                | 15 14 |
                +-------+
                | 22 23 |
                | 21 20 |
                +-------+
                | 4   5 |
                | 7   6 |
                +-------+
        '''
        s = ['' for i in range(24)]
        if filePath != None:
            f = open(filePath, 'r')
            for idx, line in enumerate(f.readlines()):
                if idx == 0:
                    s[0], s[1], s[8], s[9], s[16], s[17] = line.split()
                if idx == 1:
                    s[3], s[2], s[11], s[10], s[19], s[18] = line.split()
                if idx == 2:
                    s[12], s[13] = line.split()
                if idx == 3:
                    s[15], s[14] = line.split()
                if idx == 4:
                    s[22], s[23] = line.split()
                if idx == 5:
                    s[21], s[20] = line.split()
                if idx == 6:
                    s[4], s[5] = line.split()
                if idx == 7:
                    s[7], s[6] = line.split()
        self.config = tuple(s)

    def __hash__(self):
        return hash(self.config)
        # return hash(''.join(self.config))

    def __eq__(self, other):
        return (self.config == other.config)
        # return hash(''.join(self.config) == ''.join(other.config))

    def copy(self):
        newState = RubikState()
        newState.config = self.config
        return newState

    def isSolved(self):
        return self.config == self.goalState

    def getHeuristicValue(self):
        return (24-sum(map(operator.eq, self.config, self.goalState))) / 12.0

    def transform(self, move):
        self.parentMove = move
        ccw = move[-1] == '\''
        if ccw:
            face = move[:-1]
        else:
            face = move

        if face=='L':
            self.rotate(0, 1, 2, 3, ccw)
            self.rotate(8, 12, 22, 4, ccw)
            self.rotate(11, 15, 21, 7, ccw)
        elif face=='Ba':
            self.rotate(4, 5, 6, 7, ccw)
            self.rotate(1, 21, 17, 9, ccw)
            self.rotate(0, 20, 16, 8, ccw)
        elif face=='T':
            self.rotate(8, 9, 10, 11, ccw)
            self.rotate(1, 6, 19, 12, ccw)
            self.rotate(2, 7, 16, 13, ccw)
        elif face=='F':
            self.rotate(12, 13, 14, 15, ccw)
            self.rotate(3, 11, 19, 23, ccw)
            self.rotate(2, 10, 18, 22, ccw)
        elif face=='R':
            self.rotate(16, 17, 18, 19, ccw)
            self.rotate(9, 5, 23, 13, ccw)
            self.rotate(10, 6, 20, 14, ccw)
        elif face=='Bo':
            self.rotate(20, 21, 22, 23, ccw)
            self.rotate(5, 0, 15, 18, ccw)
            self.rotate(4, 3, 14, 17, ccw)

    def rotate(self, a, b, c, d, ccw):
        cList = list(self.config)
        if ccw == True:
            cList[a], cList[b], cList[c], cList[d] = cList[b], cList[c], cList[d], cList[a]
        else:
            cList[a], cList[b], cList[c], cList[d] = cList[d], cList[a], cList[b], cList[c]
        self.config = tuple(cList)

    def printToFile(self, filename):
        f = open(filename, 'w')
        s = self.config
        f.write(s[0] + ' ' + s[1] + ' ' + s[8] + ' ' + s[9] + ' ' + s[16] + ' ' + s[17] +'\n')
        f.write(s[3] + ' ' + s[2] + ' ' + s[11] + ' ' + s[10] + ' ' + s[19] + ' ' + s[18] +'\n')
        f.write('    ' + s[12] + ' ' + s[13] + '\n')
        f.write('    ' + s[15] + ' ' + s[14] + '\n')
        f.write('    ' + s[22] + ' ' + s[23] + '\n')
        f.write('    ' + s[21] + ' ' + s[20] + '\n')
        f.write('    ' + s[4] + ' ' + s[5] + '\n')
        f.write('    ' + s[7] + ' ' + s[6] + '\n')
        f.close()

class RubikProblem():
    faces = ['L', 'R', 'F', 'Ba', 'T', 'Bo']
    dirs = ['', '\'']

    def __init__(self, inputFile, strategy):
        self.startState = RubikState(inputFile)
        self.startState.pathCost = 0
        self.strategy = strategy

    def getNextStates(self, state):
        nextStates = []
        for face in self.faces:
            for dir in self.dirs:
                move = face + dir
                newState = state.copy()
                newState.transform(move)
                newState.parentState = hash(state)
                nextStates.append(newState)

        return nextStates

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
            h = state.getHeuristicValue()
            self.frontier.put(h, state)
        elif self.strategy=='Astar':
            h = state.getHeuristicValue()
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

    def getSolutionMoves(self):
        moves = []
        curr = self.endState
        while curr!=None:
            moves.append(curr.parentMove)
            curr = self.explored[curr.parentState]
            if curr == self.startState:
                break
        moves.reverse()
        return moves