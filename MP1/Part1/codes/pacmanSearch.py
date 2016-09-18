__author__ = 'Abhishek'
# This file contains algorithms to perform searches in the PacMan state space

from pacmanFileIO import *
from Queue import *
from sets import Set
import numpy as np
from customPriorityQueue import *

def initializeFrontier(strategy='BFS'):
    if strategy == 'BFS':
        frontier = Queue()
    elif strategy == 'DFS':
        frontier = LifoQueue()
    elif strategy == 'greedy' or strategy == 'Astar':
        frontier = customPriorityQueue()
        frontier.pq = []    # WHY IS THIS NOT NULL?!?!?!
    return frontier

def pushToFrontier_1G(pacmanStates, frontier, state, goalStates, strategy, dimensions):
    if strategy=='BFS' or strategy=='DFS':
        frontier.put(state)
    elif strategy=='greedy':
        h = getHeuristicValue_1G(pacmanStates, goalStates, state)
        frontier.put(h, pacmanLoc2ID(state, dimensions))
    elif strategy=='Astar':
        h = getHeuristicValue_1G(pacmanStates, goalStates, state)
        g = pacmanStates[state[0]][state[1]].pathCost
        frontier.put(g+h, pacmanLoc2ID(state, dimensions))
    return frontier

def pushToFrontier_MG(pacmanStates, frontier, state, goalStates, strategy, dimensions):
    if strategy=='BFS' or strategy=='DFS':
        frontier.put(state)
    elif strategy=='greedy':
        h = getHeuristicValue_MG(pacmanStates, goalStates, state)
        frontier.put(h, pacmanLoc2ID(state, dimensions))
    elif strategy=='Astar':
        h = getHeuristicValue_MG(pacmanStates, goalStates, state)
        g = pacmanStates[state[0]][state[1]].pathCost
        frontier.put(g+h, pacmanLoc2ID(state, dimensions))
    return frontier

def getFromFrontier(frontier, strategy, dimensions):
    if strategy=='BFS' or strategy=='DFS':
        return (frontier, frontier.get())
    elif strategy=='greedy' or strategy=='Astar':
        next = frontier.get()
        return(frontier, pacmanID2Loc(next, dimensions))

def getNextPacmanStates(stateMatrix, y, x, wallChar='%', goalChar='.', startChar='P'):
    deltas = [1,-1]
    actionMap = {'l':[0,-1], 'r':[0,1], 'u':[-1,0], 'd':[1,0]}
    actions = actionMap.keys()
    states = []
    for action in actionMap:
        newY = y + actionMap[action][0]
        newX = x + actionMap[action][1]
        if stateMatrix[y][x].value == wallChar:
            actions = []
        elif newX < 0 or newY < 0 or newX > len(stateMatrix[0]) or newY > len(stateMatrix) or stateMatrix[newY][newX].value == wallChar:
            actions.remove(action)
        else:
            states.append([newY, newX])
    return states

def pacmanLoc2ID(loc, dimensions):
    return loc[0]*dimensions[1] + loc[1]

def pacmanID2Loc(ID, dimensions):
    return [ID // dimensions[1], ID % dimensions[1]]

def getSolutionPath(pacmanStates, goalState, startState):
    path = []
    curr = goalState
    while curr != startState:
        path.append(curr)
        curr = pacmanStates[curr[0]][curr[1]].parent
    path.append(startState)
    return path

def getHeuristicValue_1G(pacmanStates, goalStates, currState):
    goal = goalStates[0]
    h = np.sqrt(sum((np.array(currState) - np.array(goal))**2))
    return h

def getHeuristicValue_MG(pacmanStates, goalStates, currState):
    # h = float('inf')
    h = 0
    for goal in goalStates:
        dist = abs(goal[0]-currState[0])+abs(goal[1]-currState[1])
        # h = min(h, dist)
        h = max(h, dist)
    return h

def pacmanGraphSearch(pacmanStates, goalStates, startState, dimensions, strategy='DFS', wallChar='%', goalChar='.', startChar='P', outputName='DFS_medium', outFolder='temp', makeVisualization=False):
    exploredFrames = [] #List of arrays of maze after exploration
    frontier = []
    frontier = initializeFrontier(strategy)    # Initialize Frontier
    pacmanStates[startState[0]][startState[1]].pathCost = 0   # Set path cost of root node
    frontier = pushToFrontier_1G(pacmanStates, frontier, startState, goalStates, strategy, dimensions) # Push root node to frontier

    explored = Set()    # Initialize explored set
    path = [] # Initialize path

    while 1:
        if frontier.empty():
            return -1

        # print 'Frontier before: ', frontier.queueWithLocs(dimensions)
        (frontier, currNode) = getFromFrontier(frontier, strategy, dimensions)   # Expanding the frontier
        # print 'Current Node: ', currNode, 'Frontier after: ', frontier.queueWithLocs(dimensions)

        if currNode in goalStates:  # Reached goal
            path.append(getSolutionPath(pacmanStates, goalStates, startState).reverse())
            if makeVisualization:
                # generate_video(exploredFrames, outFolder, outputName+'.mp4')
                printPath(outputName+'.txt', outFolder, pacmanStates, path, goalStates, dimensions, wallChar, goalChar, startChar)

            return (pacmanStates, exploredFrames, path)

        else:
            explored.add(pacmanLoc2ID(currNode, dimensions)) # Adding to explored set
            exploredFrames.append(visualizeProgressArray(pacmanStates, explored, goalStates, dimensions, wallChar, goalChar, startChar))

            nextNodes = getNextPacmanStates(pacmanStates, currNode[0], currNode[1], wallChar, goalChar, startChar)  # Get new states and add to frontier
            for node in nextNodes:
                if pacmanLoc2ID(node , dimensions) not in explored and node not in frontier.queue:  # If child is not in explored or queue:
                    pacmanStates[node[0]][node[1]].parent = currNode
                    pacmanStates[node[0]][node[1]].pathCost = pacmanStates[currNode[0]][currNode[1]].pathCost + 1
                    frontier = pushToFrontier_1G(pacmanStates, frontier, node, goalStates, strategy, dimensions)   # Add to frontier, update the parent and pathCost
                elif node in frontier.queue:  # Else if child is in frontier with higher cost:
                    if pacmanStates[node[0]][node[1]].pathCost > pacmanStates[currNode[0]][currNode[1]].pathCost + 1:   # Update child's parent and pathCost
                        pacmanStates[node[0]][node[1]].parent = currNode
                        pacmanStates[node[0]][node[1]].pathCost = pacmanStates[currNode[0]][currNode[1]].pathCost + 1

def pacmanGraphSearch_MG(pacmanStates, goalStates, startState, dimensions, strategy='DFS', wallChar='%', goalChar='.', startChar='P', outputName='DFS_medium', outFolder='temp', makeVisualization=False):
    goalStates0 = goalStates
    count = 0
    exploredFrames = [] #List of arrays of maze after exploration
    frontier = []
    frontier = initializeFrontier(strategy)    # Initialize Frontier
    pacmanStates[startState[0]][startState[1]].pathCost = 0   # Set path cost of root node
    frontier = pushToFrontier_MG(pacmanStates, frontier, startState, goalStates, strategy, dimensions) # Push root node to frontier

    explored = Set()    # Initialize explored set
    path = [] # Initialize path

    while 1:
        print count

        if frontier.empty():
            return -1

        (frontier, currNode) = getFromFrontier(frontier, strategy, dimensions)   # Expanding the frontier

        if currNode in goalStates:  # Reached goal
            # *** Added path to cumulative path; updated startState and GoalState, clear explored states ***
            currPath = getSolutionPath(pacmanStates, currNode, startState)
            currPath.reverse()
            path.append(currPath)
            startState = currNode
            goalStates.remove(currNode)
            explored = Set()
            frontier = initializeFrontier(strategy)
            if goalStates!=[]:
                frontier = pushToFrontier_MG(pacmanStates, frontier, currNode, goalStates, strategy, dimensions)

            if len(goalStates)==0:
                if makeVisualization:
                    exploredFrames = getMultiGoalFrames(pacmanStates, path, goalStates0, dimensions)
                    generate_video(exploredFrames, outFolder, outputName+'.mp4')
                    printPath_MG(outputName+'.txt', outFolder, pacmanStates, path)

                return (pacmanStates, exploredFrames, path, count)

        else:
            explored.add(pacmanLoc2ID(currNode, dimensions)) # Adding to explored set
            count += 1
            # exploredFrames.append(visualizeProgressArray(pacmanStates, explored, goalStates, dimensions, wallChar, goalChar, startChar))

            nextNodes = getNextPacmanStates(pacmanStates, currNode[0], currNode[1], wallChar, goalChar, startChar)  # Get new states and add to frontier
            for node in nextNodes:
                if pacmanLoc2ID(node , dimensions) not in explored and node not in frontier.queue:  # If child is not in explored or queue:
                    pacmanStates[node[0]][node[1]].parent = currNode
                    pacmanStates[node[0]][node[1]].pathCost = pacmanStates[currNode[0]][currNode[1]].pathCost + 1
                    frontier = pushToFrontier_MG(pacmanStates, frontier, node, goalStates, strategy, dimensions)   # Add to frontier, update the parent and pathCost
                elif node in frontier.queue:  # Else if child is in frontier with higher cost:
                    if pacmanStates[node[0]][node[1]].pathCost > pacmanStates[currNode[0]][currNode[1]].pathCost + 1:   # Update child's parent and pathCost
                        pacmanStates[node[0]][node[1]].parent = currNode
                        pacmanStates[node[0]][node[1]].pathCost = pacmanStates[currNode[0]][currNode[1]].pathCost + 1

def runAllAlgos_1G():
    fileNames1 = ['medium', 'big', 'open']
    algos = ['BFS', 'DFS', 'greedy', 'Astar']
    # fileNames1 = ['medium']
    # algos = ['BFS', 'Astar']
    inFolder = '../data/singleGoal'
    outFolder = 'output_1G'

    fOut = open(outFolder+'/Results_1G.txt', 'w')
    fOut.write('File\tAlgo\tExpanded\tShortest\n')

    for file in fileNames1:
        (pacmanStates, goalStates, startState, dimensions) = readPacmanFile(inFolder+'/' + file + 'Maze.txt')
        for algo in algos:
            (pacmanStates, exploredFrames, path) = pacmanGraphSearch(pacmanStates, goalStates, startState, dimensions, strategy=algo, outputName=algo+'_'+file, makeVisualization=False)
            print file, algo, len(path)-1
            fOut.write(file+'\t'+algo+'\t'+str(len(exploredFrames))+'\t\t'+str(len(path)-1)+'\n')
    fOut.close()

def runAllAlgos_MG():
    fileNames1 = ['tiny', 'small', 'medium', 'big']
    algos = ['greedy', 'Astar']
    # fileNames1 = ['big']
    # algos = ['BFS']
    inFolder = '../data/multiGoal'
    outFolder = 'output_MG'
    fOut = open(outFolder+'/Results_MG_BFS.txt', 'w')
    fOut.write('File\tAlgo\tExpanded\tShortest\n')

    for file in fileNames1:
        for algo in algos:
            (pacmanStates, goalStates, startState, dimensions) = readPacmanFile(inFolder+'/' + file + 'Search.txt')
            (pacmanStates, exploredFrames, path, count) = pacmanGraphSearch_MG(pacmanStates, goalStates, startState, dimensions, strategy=algo, outputName=algo+'_'+file, outFolder=outFolder, makeVisualization=False)
            totalLength = 0
            for x in path:
                totalLength += len(x)-1
            print file, algo, totalLength
            fOut.write(file+'\t'+algo+'\t'+str(count)+'\t\t'+str(totalLength)+'\n')
    fOut.close()

if __name__ == '__main__':
    runAllAlgos_MG()