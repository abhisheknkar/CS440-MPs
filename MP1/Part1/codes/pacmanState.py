__author__ = 'Abhishek'
# This file contains the definition for the class PacmanState, which is the state, or the knowledge about being
# at a tile in the PacMan environment

class PacmanState():
    parent = [-1,-1]
    pathCost = float('inf')
    def __init__(self, y=-1, x=-1, value='%'):
        self.y = y
        self.x = x
        self.value = value

    def setPathCost(self, pathCost):
        self.pathCost = pathCost

    def setParent(self, parent):
        self.parent = parent