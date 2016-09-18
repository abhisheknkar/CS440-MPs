__author__ = 'Abhishek'
# This file contains IO functions related to reading from and writing to files pertaining to the PacMan puzzle

from pacmanState import PacmanState
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from scipy.misc import imshow
from scipy.misc import toimage
import os
import subprocess
import glob
from pacmanSearch import *

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
            pacmanStates[y][x] = PacmanState(y, x, char)
            if char == goalChar:
                goalStates.append([y,x])
            if char == startChar:
                startState = [y,x]
    f.close()
    return (pacmanStates, goalStates, startState, [height, width-1])

def visualizeProgressArray(pacmanStates, explored, goalStates, dimensions, wallChar='%', goalChar='.', startChar='P', scale=10):
    output = np.ones((dimensions[0]*scale, dimensions[1]*scale))
    for i in range(dimensions[0]):
        for j in range(dimensions[1]):
            if pacmanStates[i][j].value == wallChar:
                output[i*scale:(i+1)*scale,j*scale:(j+1)*scale] = 0.7
            elif pacmanStates[i][j].value == goalChar:
                output[i*scale:(i+1)*scale,j*scale:(j+1)*scale] = 0
    for state in explored:
        loc = pacmanID2Loc(state, dimensions)
        output[loc[0]*scale:(loc[0]+1)*scale,loc[1]*scale:(loc[1]+1)*scale] = 0.3
    return output

def printPath(outputName, outFolder, pacmanStates, solutionPath, goalStates, dimensions, wallChar, goalChar, startChar):
    f = open(outFolder + '/' + outputName, 'w')

    for stateRow in pacmanStates:
        for state in stateRow:
            if state.value == wallChar:
                f.write(wallChar)
            elif state.value == startChar:
                f.write(startChar)
            elif state.value == goalChar:
                f.write(goalChar)
            elif [state.y, state.x] in solutionPath:
                # f.write(goalChar)
                f.write('#')
            else:
                f.write(' ')
        f.write('\n')
    f.close()

def printPath_MG(outputName, outFolder, pacmanStates, solutionPath):
    goalSequenceChar = '0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ'

    for idx, pathElement in enumerate(solutionPath):
        if idx < len(goalSequenceChar):
            pacmanStates[pathElement[-1][0]][pathElement[-1][1]].value = goalSequenceChar[idx]

    f = open(outFolder + '/' + outputName, 'w')

    for stateRow in pacmanStates:
        for state in stateRow:
            f.write(state.value)
        f.write('\n')
    f.close()

def getMultiGoalFrames(pacmanStates, path, goalStates, dimensions):
    exploredFrames = []
    for pathElement in path:
        currGoal = pathElement[-1]
        for idx, node in enumerate(pathElement):
            if idx == 0:
                continue
            explored = [pacmanLoc2ID(node, dimensions)]
            exploredFrames.append(visualizeProgressArray(pacmanStates, explored, goalStates, dimensions))
        pacmanStates[pathElement[-1][0]][pathElement[-1][1]].value = ' '
    return exploredFrames

def generate_video(img, folder, videoName):
    count = 0
    print len(img)
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