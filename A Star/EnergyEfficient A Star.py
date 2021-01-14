# This program implements an A* algorithm, where the open set is defined as a min heap for optimization

'''
Further developments possible:
    Record history of the route to learn traversability for return route
    Handle the case of impassability - not a likely situation
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

    heapq.heappush(open,(CALCULATE_H_COST(start, goal), start))              # First step of algorithm is appending the start node to the open set

    while (len(open) > 0):      # Loop runs while the open set is not empty - while there are more nodes to consider
        priority, current = heapq.heappop(open)                # Remove the current MapPosition node from the open set
        path.append(current)
        #print(current.to_string())

        if (priority == np.inf):                # If the smallest f_cost of the neighbors is infinte, the robot is stuck in a hole
            print("Failure")
            return "Failure"

        elif (current.get_x() == goal.get_x() and current.get_y() == goal.get_y()):               # If the goal has been reached,
            for i in path:
                print(i.to_string())
            print("\n\n")
            print("The points at which the robot turns:\n")
            for i in path:
                if (i.get_change() == True):
                    i.set_change(False)
                    i.get_previous().set_change(True)  # Reset to make sure that the points at the turn, not the points after the turn, have change variable true
                    print(i.get_previous().to_string())
                    turns.append(i.get_previous())
            return path

        for x in range(-1, 2):                                                          # x and y values to add to current values
            for y in range(-1, 2):
                if(x==0 and y==0):                                                      # No need to add current to open set
                    continue
                elif (x == 0 or y == 0):
                    neighbor = MapPosition(current.x + x, current.y + y)         # Create new neighbor location
                    neighbor1 = MapPosition(current.x + x, current.y + y)
                else:                                                           # If diagonal move, must still be of length 1
                    neighbor = MapPosition(current.x + x, current.y + y)
                    neighbor1 = MapPosition(current.x + x/math.sqrt(2), current.y + y/math.sqrt(2))

                if (current != start):
                    if (current.get_x() - current.get_previous().get_x() != neighbor.get_x() - current.get_x() or current.get_y() - current.get_previous().get_y() != neighbor.get_y() - current.get_y()):
                        neighbor.set_change(True)               # If the direction of motion changes, set the point after turn equal to true

                     # neighbor1 used in cases of diagonal. Robot should move to diagonal square, but the
                     # computation of g_cost pretends that the robot moved along the diagonal 1 unit.
                     # This ensures that the g_cost for all neighbors remains the same, while the h_cost uses the objective
                     # distance between the robot and the goal

                if (COST_TO_NEIGHBOR(current, neighbor) != np.inf):         # If neighboring square can be reached
                    newMovementCostToNeighbor = Decimal(current.get_g_cost()) + Decimal(distance_truncating(current, neighbor, neighbor1)) # Cost of getting to the neighbor
                else:
                    newMovementCostToNeighbor = np.inf
                if((neighbor.f_cost() == 0) or (newMovementCostToNeighbor < neighbor.get_g_cost())):    # If neighbor has not been put into open set,
                                                                                            # or if new distance to neighbor is smaller
                                                                                            # than one previously computed,
                    neighbor.set_g_cost(Decimal(newMovementCostToNeighbor))          # reset g and h costs for the neighbor
                    neighbor.set_h_cost(CALCULATE_H_COST(neighbor, goal))
                    neighbor.set_previous(current)                               # Set the current node as the neighbor's parent (for computing direction of route to find turns)
                    if((neighbor.f_cost(), neighbor) not in open):                   # If neighbor is not in open set,
                        heapq.heappush(open,(neighbor.f_cost(), neighbor))                   # add neighbor to open set


    print("Failure")
    return "Failure"            # In case that failure occurs



# Cost of traveling to a neighbor
def COST_TO_NEIGHBOR(current, neighbor):

    MASS = 22                               # Mass of the rover
    K_FRICTION = 0.01                        # Coefficient of kinetic friction
    S_FRICTION = 1                        # Coefficient of static friction
    P_MAX = 72                              # Maximum available motion power
    V = 0.35                                   # Velocity of the rover (assumed to be constant in this model)
    G = 9.81                                # Gravitational constant

    # Compute the difference in elevation of current location and its neighbor
    z = test_function(neighbor.get_x(), neighbor.get_y()) - test_function(current.get_x(), current.get_y())

    # Compute the angle of inclination
    ang_inclination = np.arctan(z/np.sqrt((neighbor.get_x() - current.get_x())**2 + (neighbor.get_y() - current.get_y())**2))

    # Compute the critical impermissible angle for uphill traversal
    critical_up = min(np.arcsin(P_MAX/(V*MASS*G*np.sqrt(K_FRICTION**2 + 1))) - np.arctan(K_FRICTION),
                      np.arctan(S_FRICTION - K_FRICTION))

    # Compute the critical breaking angle for downhill traversal
    critical_down = Decimal(-np.arctan(K_FRICTION))

    if ang_inclination > critical_up:       # If rover can't go uphill to reach neighbor
        return np.inf                                       # return infinity

    elif ang_inclination <= critical_up and ang_inclination > critical_down:            # Computation when angle of inclination is between the two critical angles
        cost = Decimal(MASS*G*np.sqrt((neighbor.get_x() - current.get_x())**2 + (neighbor.get_y() - current.get_y())**2 + z**2) *(K_FRICTION*np.cos(ang_inclination) + np.sin(ang_inclination)))
        return cost

    else:                           # Angle is less than the critical angle down, so no energy spending is required for motion
        return 0


def CALCULATE_H_COST(current, goal):

    if(current.get_x() == goal.get_x() and current.get_y() == goal.get_y()):        # If current position is the goal
        return 0                                                                    # return the cost of 0

    MASS = 22  # Mass of the rover
    K_FRICTION = 0.01  # Coefficient of kinetic friction
    S_FRICTION = 1  # Coefficient of static friction
    P_MAX = 72  # Maximum available motion power
    V = 0.35  # Velocity of the rover (assumed to be constant in this model)
    G = 9.81  # Gravitational constant

    # Compute the critical impermissible angle for uphill traversal
    critical_up = min(np.arcsin(P_MAX / (V * MASS * G * np.sqrt(K_FRICTION**2 + 1))) - np.arctan(K_FRICTION),
                         np.arctan(S_FRICTION - K_FRICTION))

    expected_cost_from = COST_TO_NEIGHBOR(current, goal)                # Expected energy cost directly from current to goal

    if expected_cost_from != np.inf:
        return Decimal(expected_cost_from)

    else:                           # If the cost is infinite, use alternative formula to allow for zigzag traversal of inclines
        expected_cost_from = Decimal(MASS*G*(K_FRICTION*np.cos(critical_up) + np.sin(critical_up))*(test_function(goal.get_x(), goal.get_y()) - test_function(current.get_x(), current.get_y()))/np.sin(critical_up))
        return  Decimal(expected_cost_from)


# Computes the cost of traversal from current to neighbor1 if the approzimated route of traversal is cut short at neighbor2
def distance_truncating(current, neighbor1, neighbor2):
    cost = COST_TO_NEIGHBOR(current, neighbor1)

    # First get rid of the part in COST_TO_NEIGHBOR from current to neighbor1 (longer neighbor) that involves the distance between the MapLocations
    z1 = test_function(neighbor1.get_x(), neighbor1.get_y()) - test_function(current.get_x(), current.get_y())
    cost /= Decimal(np.sqrt((neighbor1.get_x() - current.get_x())**2 + (neighbor1.get_y() - current.get_y())**2 + z1**2))

    # Then find the distance between the closer pair of neighbors, multiply it by longer pair's slope (slope by first multiplying then dividing)
    z2 = np.sqrt((neighbor2.get_x() - current.get_x())**2 + (neighbor2.get_y() - current.get_y())**2)
    z2 *= (test_function(neighbor1.get_x(), neighbor1.get_y()) - test_function(current.get_x(), current.get_y()))
    z2 /= np.sqrt((neighbor1.get_x() - current.get_x())**2 + (neighbor1.get_y() - current.get_y())**2)
    cost *= Decimal(np.sqrt((neighbor2.get_x() - current.get_x())**2 + (neighbor2.get_y() - current.get_y())**2 + z2**2))

    cost = Decimal(cost)
    return cost


'''Surface function for testing'''
def test_function(x, y):
    if(x > 4 and x < 6 and y > 4 and y < 6):
        return x**2 * y**2
    else:
        return (x**2 + y)/100


'''Testing ground'''
start = MapPosition(0, 0)
goal = MapPosition(10, 10)

findPath(start, goal)