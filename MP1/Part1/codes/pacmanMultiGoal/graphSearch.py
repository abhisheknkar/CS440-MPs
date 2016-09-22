__author__ = 'Abhishek'

from PacmanEssentials import *
from pacmanFileIO import *
from sets import *

def graphSearch(problem):
    problem.initializeFrontier()    # Initialize Frontier
    problem.initializeExplored()    # Initialize Explored
    problem.pushToFrontier(problem.startState) # Push root node to frontier

    count = 0
    while 1:
        count += 1
        if count % 1000 == 0:
            print count, curr.goalsLeft
        if problem.frontier.empty():    # Check if the frontier is empty, in which case there is no solution
            return (problem, -1)

        curr = problem.getFromFrontier()    # Check if a goal has been reached, in which case remove it from the goalsLeft of that node

        if len(curr.goalsLeft) == 1 and curr.location == curr.goalsLeft[0]:    # Check if the final goal state has been reached, in which case return the solution
            problem.endState = curr
            return problem

        problem.explored[hash(curr)] = curr   # Add the current node to the list of explored nodes
        problem.exploredOrderSequence.append(hash(curr))

        nextLocs = problem.getNextPacmanStates(curr)    # Get children of current node

        for loc in nextLocs:
            nextState = PacmanState(location=loc, goalsLeft=curr.goalsLeft)
            if curr.location in curr.goalsLeft:
                nextState.markGoalComplete(curr.location)   # To ensure that curr remains as is and doesn't get modified
            nextState.parent = hash(curr)   # Store just the hash. Get actual state from explored set
            nextState.pathCost = curr.pathCost + 1

            #If state is not in explored or queue, add it to frontier
            if hash(nextState) not in problem.explored and nextState not in problem.frontier.queue:
                problem.pushToFrontier(nextState)

            elif nextState in problem.frontier.queue:
                oldNextState = problem.frontierDetails[nextState]
                if nextState.pathCost < oldNextState.pathCost:          # If in frontier with higher path cost, replace
                    print curr.location, nextState.location
                    problem.frontierDetails[nextState] = nextState.pathCost
                    problem.pushToFrontier(nextState)

def runAllAlgos_1G():
    # fileNames = ['medium', 'big', 'open']
    # algos = ['BFS', 'DFS', 'greedy', 'Astar']
    fileNames = ['medium']
    algos = ['BFS']
    inFolder = '../../data/singleGoal'
    outFolder = 'output_1G'

    fOut = open(outFolder+'/Results_1G.txt', 'w')
    fOut.write('File\t\tStrategy\tExpanded\tShortest\n')

    for file in fileNames:
        (grid, goalStates, startState) = readPacmanFile(inFolder+'/' + file + 'Maze.txt')
        for algo in algos:
            problem = PacmanProblem(grid, startState, goalStates, algo, visualization=True)
            problem.outputName = outFolder + '/' + algo + '_' + file
            problem = graphSearch(problem)

            printPath1G(problem, outFolder + '/' + file + '_' + algo + '.txt')
            generate_video(problem, outFolder+'/shortest/', algo + '_' + file + '.mp4', mode='shortest')
            generate_video(problem, outFolder+'/exploration/', algo + '_' + file + '.mp4', mode='exploration')

            toWrite = file+'Maze.txt\t'+algo+'\t\t'+str(len(problem.exploredOrderSequence))+'\t\t'+str(len(problem.getPathCoordinates())-1)+'\n'
            print toWrite
            fOut.write(toWrite)
    fOut.close()

def runAllAlgos_MG():
    # fileNames = ['tiny', 'small', 'medium', 'big']
    # algos = ['greedy', 'Astar']
    fileNames = ['micro']
    algos = ['Astar']
    inFolder = '../../data/multiGoal'
    outFolder = 'output_MG'
    fOut = open(outFolder+'/Results_MG.txt', 'w')
    fOut.write('File\t\tStrategy\tExpanded\tShortest\n')

    for file in fileNames:
        (grid, goalStates, startState) = readPacmanFile(inFolder+'/' + file + 'Search.txt')
        for algo in algos:
            problem = PacmanProblem(grid, startState, goalStates, algo, visualization=True)
            problem.outputName = outFolder + '/' + algo + '_' + file
            problem = graphSearch(problem)

            problem.getPathCoordinates()
            printPathMG(problem, outFolder + '/' + file + '_' + algo + '.txt')
            generate_video(problem, outFolder, algo + '_' + file + '.mp4', mode='shortest')

            toWrite = file+'Search\t'+algo+'\t\t'+str(len(problem.exploredOrderSequence))+'\t\t'+str(len(problem.getPathCoordinates())-1)+'\n'
            print toWrite
            fOut.write(toWrite)
    fOut.close()

if __name__ == '__main__':
    runAllAlgos_MG()