''' Denis Rozhnov
    SBRT Software Team: Autonavigation

 This class will be used to create and test the program that allows us to find energy-efficient paths on knnown uneven terrains
 This is a precursory algorithm. It is used for testing purposes. Does not send actual commands to the motors.
 The focus of this technique is on energy efficiency, which means that it may attempt to traverse certain obstacles
 instead of avoiding them

 This is an A* algorithm
 No CLOSED set to allow for returning to previous location

 Further improvements:
      Send commands to motors
      Ensure that the robot passes near certain checkpoints
      Use input data from sensors and computer vision
      Represent obstacles as very steep inclines

'''

import queue
from MapLocation import *
import numpy as np


# Function for finding/exploring the path
def PATH_FINDER(ns, ng):

    cost_from_start = 0                     # Total cost from start

    OPEN = queue.Queue()                    # Queue of locations currently under consideration (neighbors of interest, determined later)
    OPEN.put(ns)                            # Put the starting position into the queue

    expected_cost = CALCULATE_FROM_COST(ns, ng)             # Expected cost for starting point to goal

    current = ns                                            # Preset current to the starting position

    while not OPEN.empty():

        # Calculate the cost from the first neighbor to the goal
        if (current is not ns) or (current is ns and len(OPEN.queue) > 1):                           # If current position is not starting position
            expected_cost = EXPECTED_COST(current, OPEN.queue[0], ng, cost_from_start)          # Change calculation of expected cost

            for i in OPEN.queue:                                            # By looping through the queue
                if EXPECTED_COST(current, i, ng, cost_from_start) <= expected_cost:
                    current = i                                             # identify the neighbor with the lowest expected cost
                                                                        # set current to it
                    expected_cost = EXPECTED_COST(current, i, ng, cost_from_start)           # Update expected cost

        if expected_cost == np.inf:                 # Error message if the minimal expected cost from a neighbor to the goal is infinite
            return "Path not found."

        elif current == ng:
            #return(CONSTRUCT_PATH(current.previous, ng))                # Construct the path once the goal is reached
            return CONSTRUCT_PATH(ng)

        OPEN.get(current)                               # Remove the current location from the queue


        # Define the neighbors of the current location
        neighbors =[None]*8                       # List of neighbors

        neighbors[0] = MapLocation(current.get_x(), current.get_y()+1)
        neighbors[1] = MapLocation(current.get_x()+1, current.get_y())
        neighbors[2] = MapLocation(current.get_x(), current.get_y()-1)
        neighbors[3] = MapLocation(current.get_x()-1, current.get_y())              # Creating each neighbor
        neighbors[4] = MapLocation(current.get_x()+1, current.get_y()+1)
        neighbors[5] = MapLocation(current.get_x()-1, current.get_y()+1)
        neighbors[6] = MapLocation(current.get_x()+1, current.get_y()-1)
        neighbors[7] = MapLocation(current.get_x()-1, current.get_y()-1)

        for n in neighbors:
            cost_from_start_temp =  cost_from_start + COST_TO_NEIGHBOR(current, n)      # Projected cost from start to neighbor
            total_energy_cost_temp = cost_from_start_temp + CALCULATE_FROM_COST(n, ng)  # Projected total energy if path from current
            if EXPECTED_COST(current, n, ng, cost_from_start) is None or total_energy_cost_temp < expected_cost:
                n.set_previous(current)                         # For each neighbor of interest, set its previous to current location
                cost_from_start = cost_from_start_temp          # Update cost from start
                expected_cost = total_energy_cost_temp          # Update expected cost
                if n not in OPEN.queue:
                    OPEN.put(n)                                 # Add neighbor of interest to OPEN if it is not already there

    return "Path not found."



# Function for recursively constructing the path at the end of exploration
#def CONSTRUCT_PATH(previous, goal):
 #   if previous.previous is not None:
  #      p = CONSTRUCT_PATH(previous.previous, previous)             # If previous is not the starting location
   #     return p + ", " + previous.to_string()                      # recursively return path followed by previous
    #else:                                                           # If previous is the starting location
     #   return previous.to_string() + ", "                          # return it - this is the stopping condition


def CONSTRUCT_PATH(goal):
    path = goal.to_string()
    while goal.previous is not None:
        goal = goal.get_previous()
        path = goal.to_string() + ", " + path

    return path


# Cost of traveling to a neighbor
def COST_TO_NEIGHBOR(current, neighbor):

    MASS = 50                               # Mass of the rover
    K_FRICTION = 0.2                        # Coefficient of kinetic friction
    S_FRICTION = 0.3                        # Coefficient of static friction
    P_MAX = 10                              # Maximum available motion power
    V = 2                                   # Velocity of the rover (assumed to be constant in this model)
    G = 9.81                                # Gravitational constant

    # Compute the difference in elevation of current location and its neighbor
    z = test_function(neighbor.get_x(), neighbor.get_y()) - test_function(current.get_x(), current.get_y())

    # Compute the angle of inclination
    ang_inclination = np.arctan(z/np.sqrt((neighbor.get_x() - current.get_x())**2 + (neighbor.get_y() - current.get_y())**2))

    # Compute the critical impermissible angle for uphill traversal
    critical_up = min(np.arcsin(P_MAX/(V*MASS*G*np.sqrt(K_FRICTION**2 + 1))) - np.arctan(K_FRICTION), np.arctan(S_FRICTION - K_FRICTION))

    # Compute the critical breaking angle for downhill traversal
    critical_down = -np.arctan(K_FRICTION)

    if ang_inclination > critical_up:                       # If rover can't go uphill to reach neighbor
        return np.inf                                       # return infinity

    elif ang_inclination <= critical_up and ang_inclination > critical_down:            # Computation when angle of inclination is between the two critical angles
        cost = MASS*G*np.sqrt((neighbor.get_x() - current.get_x())**2 + (neighbor.get_y() - current.get_y())**2 + z**2) + z**2*(K_FRICTION*np.cos(ang_inclination) + np.sin(ang_inclination))
        return cost

    else:                           # Angle is less than the critical angle down, so no energy spending is required for motion
        return 0


def CALCULATE_FROM_COST(current, goal):

    MASS = 50  # Mass of the rover
    K_FRICTION = 0.2  # Coefficient of kinetic friction
    S_FRICTION = 0.3  # Coefficient of static friction
    P_MAX = 10  # Maximum available motion power
    V = 2  # Velocity of the rover (assumed to be constant in this model)
    G = 9.81  # Gravitational constant

    # Compute the critical impermissible angle for uphill traversal
    critical_up = min(np.arcsin(P_MAX / (V * MASS * G * np.sqrt(K_FRICTION ** 2 + 1))) - np.arctan(K_FRICTION),
                         np.arctan(S_FRICTION - K_FRICTION))

    expected_cost_from = COST_TO_NEIGHBOR(current, goal)                # Expected energy cost directly from current to goal

    if expected_cost_from != np.inf:
        return expected_cost_from

    else:                           # If the cost is infinite, use alternative formula to allow for zigzag traversal of inclines
        expected_cost_from = MASS*G*(K_FRICTION*np.cos(critical_up) + np.sin(critical_up))*(test_function(goal.get_x(), goal.get_y()) - test_function(current.get_x(), current.get_y()))/np.sin(critical_up)
        return  expected_cost_from


# Function to compute the expected cost of passing through the neighbor
def EXPECTED_COST(current, neighbor, goal, cost_from_start):
    return CALCULATE_FROM_COST(neighbor, goal) + COST_TO_NEIGHBOR(current, neighbor) + cost_from_start

# Function for testing the algorithm - simulation of terrain
def test_function(x, y):
    return 0



# This is the test run of the algorithm

start = MapLocation(0, 0)                   # Starting location
goal = MapLocation(10, 10)                  # Goal location

print(PATH_FINDER(start, goal))                    # Should be a recursive call of PATH_FINDER
