# This program implements a basic A* algorithm

from MapPosition import MapPosition
import math
import decimal
from decimal import *
getcontext().prec = 5

def findPath(start, goal):
    open = []                       # open set to store MapPosition nodes under consideration
    closed = []                     # closed set to store MapPosition nodes no longer under consideration

    open.append(start)              # First step of algorithm is appending the start node to the open set

    while (len(open) > 0):      # Loop runs while the open set is not empty - while there are more nodes to consider
        current = open[0]           # Set the current MapPosition pointer to the first element of the open set

        for mPosition in open:      # Loop through the open set, look at each MapPosition (mPosition)
            #print(str(mPosition.x) + ", " + str(mPosition.y) + ", " + str(mPosition.get_h_cost()))
            #if (len(open) == 12):
             #   print(mPosition.f_cost())
              #  print(mPosition.get_h_cost())
               # print("\n")
            if(mPosition.f_cost() < current.f_cost()):      # If the mPosition's f_cost is less than the current node's f_cost,
                current = mPosition                                 # set the current node to mPosition (to minimize the f_cost)
                                                                    # In this way you pick the optimal next step
                #print(current.to_string())
            elif(mPosition.f_cost() == current.f_cost() and mPosition.get_h_cost() < current.get_h_cost()):
                current = mPosition                                 # Do the same if the f_cost of a MapPosition in the open set
                                                                    # is equal to the current node's f_cost but it has a lower
                                                                    # heuristic h_cost than that of the current node
 #               print(current.to_string())

        open.remove(current)                # Remove the current MapPosition node from the open set
        closed.append(current)              # Add it to the closed set, since it will no longer be considered
        #print(str(current.x) + ", " + str(current.y))
        # Each entry appended to the closed list becomes a command for the robot

        if (current.x == goal.x and current.y == goal.y):               # If the goal has been reached,
            for i in range(len(closed)):
                print(closed[i].to_string())        # Print path
            return closed                         # Exit out of the loop.

        for x in range(-1, 2):                                                          # x and y values to add to current values
            for y in range(-1, 2):
                if(x==0 and y==0):                                                      # No need to add current to open set
                    continue
                neighbor = MapPosition(current.x + x, current.y + y)        # Create new neighbor location
                traverse(neighbor)

                if(neighbor.get_walkable() == False or neighbor in closed):             # If neighbor is not walkable or is in closed,
                    continue                                                            # ignore the neighbor

                newMovementCostToNeighbor = current.get_g_cost() + getDistance(current, neighbor) # Cost of getting to the neighbor
                if((neighbor not in open) or (newMovementCostToNeighbor < neighbor.get_g_cost())):    # If neighbor has not been put into open set,
                                                                                            # or if new distance to neighbor is smaller
                                                                                            # than one previously computed,
                    neighbor.set_g_cost(Decimal(newMovementCostToNeighbor))          # reset g and h costs for the neighbor
                    neighbor.set_h_cost(getDistance(neighbor, goal))
#                    neighbor.previous = current                               # Set the current node as the neighbor's parent
                    if(neighbor not in open):  # If neighbor is not in open set,
                        open.append(neighbor)  # add neighbor to open set

# The following getDistance method might be modified when we integrate the physics computations into our model

def getDistance(a, b):                          # Finds the distance between two nodes
    distX = abs(a.get_x() - b.get_x())                 # x and y coordinate differences between the two nodes
    distY = abs(a.get_y() - b.get_y())

    if(distX > distY):
        return Decimal(math.sqrt(2)*distY + distX - distY)           # sqrt(2) used for extra precision of distance, units to be determined
    return Decimal(math.sqrt(2)*distX + distY - distX)



'''Traversability function for testing'''
def traverse(mapNode):
    if (mapNode.get_x() == 2 and mapNode.get_y() == 2):
        mapNode.set_walkable(False)

    return



'''Testing ground'''
start = MapPosition(0, 0)
goal = MapPosition(10, 27)

findPath(start, goal)