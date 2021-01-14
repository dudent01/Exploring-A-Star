# This program implements an A* algorithm, where the open set is defined as a min heap for optimization

from MapPosition import MapPosition
import math
import heapq                        # Min heap library
from decimal import *
getcontext().prec = 5               # Sets precision to 5 decimal places to avoid rounding error

def findPath(start, goal):
    open = []                       # open set to store MapPosition nodes under consideration
    closed = []                     # closed set to store MapPosition nodes no longer under consideration

    heapq.heappush(open,(0, start))              # First step of algorithm is appending the start node to the open set

    while (len(open) > 0):      # Loop runs while the open set is not empty - while there are more nodes to consider
        priority, current = heapq.heappop(open)                # Remove the current MapPosition node from the open set
        closed.append(current)              # Add it to the closed set, since it will no longer be considered
        #print(str(current.x) + ", " + str(current.y))

        if (current.get_x() == goal.get_x() and current.get_y() == goal.get_y()):               # If the goal has been reached,
            #retracePath(start, goal)        # Call the retracePath function
            for i in closed:
                print(i.to_string())
            return closed                        # Exit out of the loop

        for x in range(-1, 2):                                                          # x and y values to add to current values
            for y in range(-1, 2):
                if(x==0 and y==0):                                                      # No need to add current to open set
                    continue
                neighbor = MapPosition(current.x + x, current.y + y)        # Create new neighbor location
                traverse(neighbor)

                if(neighbor.get_walkable() == False or neighbor in closed):             # If neighbor is not walkable or is in closed,
                    continue                                                            # ignore the neighbor

                newMovementCostToNeighbor = current.get_g_cost() + getDistance(current, neighbor) # Cost of getting to the neighbor
                if(((neighbor.f_cost(), neighbor) not in open) or (newMovementCostToNeighbor < neighbor.get_g_cost())):    # If neighbor has not been put into open set,
                                                                                            # or if new distance to neighbor is smaller
                                                                                            # than one previously computed,
                    neighbor.set_g_cost(Decimal(newMovementCostToNeighbor))          # reset g and h costs for the neighbor
                    neighbor.set_h_cost(getDistance(neighbor, goal))
                    #neighbor.set_previous(current)                               # Set the current node as the neighbor's parent
                    if((neighbor.f_cost(), neighbor) not in open):                   # If neighbor is not in open set,
                        heapq.heappush(open,(neighbor.f_cost(), neighbor))                   # add neighbor to open set

# The following getDistance method might be modified when we integrate the physics computations into our model

def getDistance(a, b):                          # Finds the distance between two nodes
    distX = abs(a.x - b.x)                 # x and y coordinate differences between the two nodes
    distY = abs(a.y - b.y)

    if(distX > distY):
        return Decimal(math.sqrt(2)*distY + distX - distY)           # sqrt(2) used for extra precision of distance, units to be determined
    return Decimal(math.sqrt(2)*distX + distY - distX)


#def retracePath(start, end):            # Function that retraces and returns the path
 #   path = []                           # path list stores the list of nodes
  #  currentNode = end                   # Set the currentNode MapPosition node pointer to the end MapPosition node
   # path.append(currentNode)

#    while(currentNode.get_x() != start.get_x() and currentNode.get_y() != start.get_y()):        # While the start node has not been reached
 #       currentNode = currentNode.previous    # Reset the current node to the current node's parent
  #      path.append(currentNode)        # Add the current node to the path list


#    path.reverse()              # Reverse the order of the path, since it is currently in reverse order

#    for node in path:
 #       node.to_string()

#    return path


'''Traversability function for testing'''
def traverse(mapNode):
    if (mapNode.get_x() == 2 and mapNode.get_y() == 2):
        mapNode.set_walkable(False)

    return



'''Testing ground'''
start = MapPosition(0, 0)
goal = MapPosition(10, 12)

findPath(start, goal)