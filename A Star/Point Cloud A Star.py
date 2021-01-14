# This program implements our finalized A* algorithm

'''
For this algorithm, we optimized A* by using a minheap instead of a list for the open set, to speed up insertion and deletion.
We also did not utilize the closed set, allowing the rover to return to a previous location if need be.

For our heuristic we used a measure of danger of traversal to an adjacent square. This measure of danger is based on the
relative elevations of the two grid locations (MapPositions). The height in this case takes two potential values - the
20'th percentile and the 80'th percentile of the z values of the ten most recent points computed at the MapPosition
under consideration. The worst case height difference is taken as a danger metric for traversing between two different
MapPositions. We use this measure to run the A* algorithm.

When we determine the optimal path to take, we also ensure that the route is also physically feasible by computing the critical
angle of ascent or descent.
'''

from MapPosition import MapPosition
import heapq                        # Min heap library
import math
import numpy as np
from decimal import *
getcontext().prec = 5               # Sets precision to 5 decimal places to avoid rounding errors


def findPath(start, goal):
    open = []                       # open set to store MapPosition nodes under consideration
    # No closed set because the same node can be potentially revisited
    path = []
    turns = []                      # List of points at which the robot turns