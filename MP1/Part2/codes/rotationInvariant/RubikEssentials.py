__author__ = 'Abhishek'
import numpy as np
import operator
from Queue import *
from customPriorityQueue import *

class RubikState:
    parentState = None
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
        newState.parentState = self.parentState
        newState.parentMove = self.parentMove
        newState.pathCost = self.pathCost
        return newState

    def transform(self, move):
        self.parentMove = move
        ccw = move[-1] == '\''
        if ccw:
            face = move[:-1]
        else:
            face = move

        if face=='L':
            self.turn(0, 1, 2, 3, ccw)
            self.turn(8, 12, 22, 4, ccw)
            self.turn(11, 15, 21, 7, ccw)
        elif face=='Ba':
            self.turn(4, 5, 6, 7, ccw)
            self.turn(1, 21, 17, 9, ccw)
            self.turn(0, 20, 16, 8, ccw)
        elif face=='T':
            self.turn(8, 9, 10, 11, ccw)
            self.turn(1, 6, 19, 12, ccw)
            self.turn(2, 7, 16, 13, ccw)
        elif face=='F':
            self.turn(12, 13, 14, 15, ccw)
            self.turn(3, 11, 19, 23, ccw)
            self.turn(2, 10, 18, 22, ccw)
        elif face=='R':
            self.turn(16, 17, 18, 19, ccw)
            self.turn(9, 5, 23, 13, ccw)
            self.turn(10, 6, 20, 14, ccw)
        elif face=='Bo':
            self.turn(20, 21, 22, 23, ccw)
            self.turn(5, 0, 15, 18, ccw)
            self.turn(4, 3, 14, 17, ccw)

    def turn(self, a, b, c, d, ccw):
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

    def rotate(self, id):
        # ID=0: Top-bottom, CCW from Top
        # ID=1: Front-back, CW from Front
        # ID=2: Left-Right, CW from Left
        c = self.config
        if id == 0:
            self.config = (c[5], c[6], c[7], c[4],\
                           c[17], c[18], c[19], c[16],\
                           c[9], c[10], c[11], c[8],\
                           c[1], c[2], c[3], c[0],\
                           c[13], c[14], c[15], c[12],\
                           c[23], c[20], c[21], c[22])

        if id == 1:
            self.config = (c[20], c[21], c[22], c[23],\
                           c[5], c[6], c[7], c[4],\
                           c[0], c[1], c[2], c[3],\
                           c[15], c[12], c[13], c[14],\
                           c[8], c[9], c[10], c[11],\
                           c[16], c[17], c[18], c[19])

        if id == 2:
            self.config = (c[3], c[0], c[1], c[2],\
                           c[22], c[23], c[20], c[21],\
                           c[4], c[5], c[6], c[7],\
                           c[8], c[9], c[10], c[11],\
                           c[17], c[18], c[19], c[16],\
                           c[14], c[15], c[12], c[13])

    def getDegenerateStates(self):
        degenerates = []
        degenerates.append(self)
        curr = self
        for i in range(3):
            curr = curr.copy()
            curr.rotate(0)
            degenerates.append(curr)

        curr = self
        for i in range(2):
            curr = curr.copy()
            if i == 1:
                curr.rotate(2)
            curr.rotate(2)
            degenerates.append(curr)

        baseDegens = degenerates[:]
        for elem in baseDegens:
            curr = elem
            for i in range(3):
                curr = curr.copy()
                curr.rotate(1)
                degenerates.append(curr)
        return degenerates

    def getEqualPairs(self, L):
        count = 0
        for x in L:
            for y in L:
                if x == y:
                    count += 1
        return count

class RubikProblem():
    goalStateConfig = ('r', 'r', 'r', 'r', 'p', 'p', 'p', 'p', 'b', 'b', 'b', 'b', 'o', 'o', 'o', 'o', 'g', 'g', 'g', 'g', 'y', 'y', 'y', 'y')
    faces = ['L', 'R', 'F', 'Ba', 'T', 'Bo']
    dirs = ['', '\'']

    def __init__(self, inputFile, strategy='Astar'):
        self.startState = RubikState(inputFile)
        self.startState.pathCost = 0
        self.strategy = strategy

        x = RubikState()
        x.config = self.goalStateConfig
        self.goalStates = tuple(x.getDegenerateStates())

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

    def getSolutionMoves(self):
        moves = []
        curr = self.endState

        while curr!=None:
            moves.append(curr.parentMove)
            if curr.parentState!=None:
                curr = self.explored[curr.parentState]
            if curr == self.startState:
                break
        moves.reverse()
        return moves

    def isSolved(self, state):
        return state in self.goalStates

    def getHeuristicValue(self, state):
        h = float('inf')
        for goalState in self.goalStates:
            h = min(h, (24-sum(map(operator.eq, state.config, goalState.config))) / 12.0)
        return h

if __name__ == '__main__':
    r = RubikState('../../data/cube0.txt')
    print r.getEqualPairs(r.getDegenerateStates())
    # r.rotate(2)
    # r.printToFile('RotateLR.txt')