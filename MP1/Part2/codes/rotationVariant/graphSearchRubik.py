__author__ = 'Abhishek'

from RubikEssentials import *
import time

def graphSearch(problem):
    problem.initializeFrontier()    # Initialize Frontier
    problem.initializeExplored()    # Initialize Explored
    problem.pushToFrontier(problem.startState) # Push root node to frontier

    count = 0
    while 1:
        count += 1
        # if count % 100 == 0:
        #     print count, curr.pathCost, curr.getHeuristicValue()

        if problem.frontier.empty():    # Check if the frontier is empty, in which case there is no solution
            return -1

        curr = problem.getFromFrontier()    # Check if a goal has been reached, in which case remove it from the goalsLeft of that node

        if curr.isSolved():    # Check if the final goal state has been reached, in which case return the solution
            problem.endState = curr
            return problem

        problem.explored[hash(curr)] = curr   # Add the current node to the list of explored nodes
        problem.exploredOrderSequence.append(hash(curr))

        nextStates = problem.getNextStates(curr)    # Get children of current node

        for nextState in nextStates:
            nextState.parentState = hash(curr)   # Store just the hash. Get actual state from explored set
            nextState.pathCost = curr.pathCost + 1

            #If state is not in explored or queue, add it to frontier
            if hash(nextState) not in problem.explored and nextState not in problem.frontierDetails:
                problem.pushToFrontier(nextState)

            # elif nextState in problem.frontier.queue:
            elif nextState in problem.frontierDetails:
                oldNextState = problem.frontierDetails[nextState]
                if nextState.pathCost < oldNextState.pathCost:          # If in frontier with higher path cost, replace
                    problem.frontierDetails[nextState] = nextState.pathCost
                    problem.pushToFrontier(nextState)

def makeNewPerms(problem):
    state = problem.startState
    state.transform('F')
    state.transform('L')

    state.transform('Bo')
    state.transform('Ba')
    state.transform('R')
    state.transform('T')
    state.printToFile('out6.txt')

if __name__ == '__main__':
    fOut = open('Results.txt', 'w')
    fOut.write('File\t\tExpanded\tPath\t\t\t\t\tTime\n')
    for i in range(3):
        print 'Solving cube1_'+str(i+1)+'.txt'
        t1 = time.time()
        problem = RubikProblem(inputFile='../../data/cube1_'+str(i+1)+'.txt')
        problem = graphSearch(problem)
        t2 = time.time()
        if problem == -1:
            print 'No solution found'
        else:
            fOut.write('cube1_'+str(i+1)+'.txt\t'+str(len(problem.explored))+'\t\t'+str(problem.getSolutionMoves())+'\t\t'+str(t2-t1)+'\n')
    fOut.close()
