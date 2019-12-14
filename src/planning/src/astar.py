from Queue import PriorityQueue
import numpy as np


class Search:
    def __init__(self, map, start, goal):
        """
        assume that the map is a 2D array of booleans, where True means wall
        defined as x pointing downward and y point to the right
        """
        self.map = map
        self.start = start
        self.goal = goal
        self.path = []

    def isGoal(self, pos):
        """
        check if the given position is equal to the GOAL
        """
        return pos == self.goal

    def successors(self, pos):
        """
        return a list of legal positions of next step
        """
        legalMoves = []
        x, y = pos
        # to the top
        if self.isLegal(x - 1, y):
            legalMoves.append([x - 1, y])
        # to the bottom
        if self.isLegal(x + 1, y):
            legalMoves.append([x + 1, y])
        # to the left
        if self.isLegal(x, y - 1):
            legalMoves.append([x, y - 1])
        # to the right
        if self.isLegal(x, y + 1):
            legalMoves.append([x, y + 1])

        # to the top left corner
        if self.isLegal(x - 1, y) and self.isLegal(x - 1, y - 1) and self.isLegal(x, y - 1):
            legalMoves.append([x - 1, y - 1])
        # to the top right corner
        if self.isLegal(x - 1, y) and self.isLegal(x - 1, y + 1) and self.isLegal(x, y + 1):
            legalMoves.append([x - 1, y + 1])
        # to the bottom left corner
        if self.isLegal(x + 1, y) and self.isLegal(x + 1, y - 1) and self.isLegal(x, y - 1):
            legalMoves.append([x + 1, y - 1])
        # to the bottom right corner
        if self.isLegal(x + 1, y) and self.isLegal(x + 1, y + 1) and self.isLegal(x, y + 1):
            legalMoves.append([x + 1, y + 1])

        return legalMoves


    def isLegal(self, x, y):
        """
        check if the position x and y is inside the map
        and if the position is not a wall
        """
        return x >= 0 and x < len(self.map) and y >= 0 and y < len(self.map[0])\
                and not self.map[x][y]

    def euclideanHeuristic(self, pos):
        """
        return the euclidean distance between the POS and the goal
        """
        return ((self.goal[0] - pos[0]) ** 2 + (self.goal[1] - pos[1]) ** 2) ** 0.5

    def astar(self):
        """
        apply astar to find the shortest path from start to goal
        return a list of positions
        """
        fringe = PriorityQueue()
        closed = []
        fringe.put((0, Node(self.start)))
        while not fringe.empty():
            _, node = fringe.get()
            if self.isGoal(node.position):
                return [self.start] + node.path
            if node.position not in closed:
                closed.append(node.position)
                for child in self.successors(node.position):
                    fringe.put((node.backGroundCost + self.euclideanHeuristic(child), \
                    Node(child, node.path + [child], node.backGroundCost + 1)))
        return []


    def pathProcess(self):
        """
        pre-process the path returned by astar or any other method, delete positions at which orientation does not change
        """
        tempPath = self.astar()
        if len(tempPath) < 3:
            self.path = tempPath
        else:
            curr, next, diff = 0, 1, (0, 0)
            while next < len(tempPath):
                newDiff = self.difference(tempPath[next], tempPath[curr])
                if newDiff != diff:
                    self.path.append(tempPath[curr])
                    diff = newDiff
                curr = next
                next += 1
            self.path.append(tempPath[curr])

    def difference(self, next, curr):
        """
        return a tuple of the x, y coordinate difference between CURR position and NEXT position
        """
        return next[0] - curr[0], next[1] - curr[1]

    def findPath(self):
        """
        return the pre-processed path from START to GOAL
        """
        self.pathProcess()
        return self.path

class Node:
    """
    graph node represent a state
    which stores three values: the location, path from the starting location
    and backGroundCost from the starting location
    """
    def __init__(self, pos, path = [], cost = 0):
        self.position = pos
        self.path = path
        self.backGroundCost = cost
