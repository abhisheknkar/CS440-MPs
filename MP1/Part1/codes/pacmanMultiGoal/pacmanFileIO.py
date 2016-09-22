__author__ = 'Abhishek'
# This file contains IO functions related to reading from and writing to files pertaining to the PacMan puzzle

from PacmanEssentials import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from scipy.misc import imshow
from scipy.misc import toimage
import os
import subprocess
import glob

def readPacmanFile(filepath, wallChar='%', goalChar='.', startChar='P'):
    f = open(filepath, 'r')
    lines = f.readlines()
    for y, line in enumerate(lines):
        for x, char in enumerate(line):
            if y == 0 and x == 0:
                height = len(lines)
                width = len(line)
                pacmanStates = [[0 for i in range(width-1)] for j in range(height)]
                goalStates = []
            if char == '\n':
                continue
            pacmanStates[y][x] = PacmanState((y, x), value=char)
            if char == goalChar:
                goalStates.append((y,x))
            if char == startChar:
                startState = (y,x)
    f.close()
    return (pacmanStates, tuple(goalStates), startState)

def visualizeExplorationSequence(problem, scale=10):
    explorationSequence = []
    frame = np.ones((problem.gridHeight*scale, problem.gridWidth*scale))
    for i in range(problem.gridHeight):
        for j in range(problem.gridWidth):
            if problem.grid[i][j].value == problem.wallChar:
                frame[i*scale:(i+1)*scale,j*scale:(j+1)*scale] = 0.7
            elif problem.grid[i][j].location in problem.goalStates:
                frame[i*scale:(i+1)*scale,j*scale:(j+1)*scale] = 0
    for stateHash in problem.exploredOrderSequence:
        state = problem.explored[stateHash]
        loc = state.location
        frame[loc[0]*scale:(loc[0]+1)*scale,loc[1]*scale:(loc[1]+1)*scale] = 0.3
        explorationSequence.append(frame.copy())
    return explorationSequence

def printPath1G(problem, outFile):
    f = open(outFile, 'w')
    path = problem.getPathCoordinates()
    for y in range(problem.gridHeight):
        for x in range(problem.gridWidth):
            if (y,x) == problem.startState.location:
                f.write(problem.startChar)
            elif (y,x) in path:
                f.write('#')
            else:
                f.write(problem.grid[y][x].value)
        f.write('\n')
    f.close()

def printPathMG(problem, outFile):
    goalSequenceChar = '0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ'
    f = open(outFile, 'w')
    path = problem.getPathCoordinates()
    goalsLeft = list(problem.goalStates)

    count = 0
    for pathElement in path:
        if pathElement in goalsLeft:
            goalsLeft.remove(pathElement)
            gridCopy = list(problem.grid)
            gridCopy[pathElement[0]][pathElement[1]].value = goalSequenceChar[count]
            count += 1

    for row in gridCopy:
        for elem in row:
            f.write(elem.value)
        f.write('\n')
    f.close()

def visualizeShortestSequence(problem, scale=10):
    shortestSequence = []
    path = problem.getPathCoordinates()

    frame = np.ones((problem.gridHeight*scale, problem.gridWidth*scale))
    for i in range(problem.gridHeight):
        for j in range(problem.gridWidth):
            if problem.grid[i][j].value == problem.wallChar:
                frame[i*scale:(i+1)*scale,j*scale:(j+1)*scale] = 0.7
            elif problem.grid[i][j].location in problem.goalStates:
                frame[i*scale:(i+1)*scale,j*scale:(j+1)*scale] = 0

    prevLoc = ()
    for idx, loc in enumerate(path):
        if len(prevLoc)>0:
            frame[prevLoc[0]*scale:(prevLoc[0]+1)*scale,prevLoc[1]*scale:(prevLoc[1]+1)*scale] = 1
        frame[loc[0]*scale:(loc[0]+1)*scale,loc[1]*scale:(loc[1]+1)*scale] = 0.3
        prevLoc = loc
        shortestSequence.append(frame.copy())
    return shortestSequence

def generate_video(problem, folder, videoName, mode='exploration'):
    if mode == 'exploration':
        img = visualizeExplorationSequence(problem)
    elif mode == 'shortest':
        img = visualizeShortestSequence(problem)
    count = 0
    for i in xrange(len(img)):
        count += 1
        # plt.imshow(img[i], cmap=cm.Greys_r)
        # plt.savefig(folder + "/file%03d.png" % i)
        toimage(img[i]).save(folder + "/file%03d.png" % i)

    cwd = os.getcwd()

    os.chdir(folder)
    subprocess.call('ffmpeg -r 10 -i file%03d.png '+str(videoName), shell=True)
    # subprocess.call('ffmpeg -r 10 -i file%03d.png -vf scale=120:-1 '+str(videoName), shell=True)

    for file_name in glob.glob("*.png"):
        os.remove(file_name)
    os.chdir(cwd)