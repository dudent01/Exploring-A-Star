// Denis Rozhnov
// SBRT Software Team: Autonavigation

// This class will be used to create and test the program that allows us to find energy-efficient paths on knnown uneven terrains
// This is a precursory algorithm. It is used for testing purposes. Does not send actual commands to the motors.

// Further improvements:
//      Send commands to motors
//      Ensure that the robot passes near certain checkpoints
//      Use input data from sensors and computer vision
//      Represent obstacles as very steep inclines

import java.util.LinkedList;
import java.util.Queue;

public class EnergyEfficient {
    /**
     * Main method for testing the algorithm
     * @param args
     */
    public static void main(String[] args){
        MapLocation start = new MapLocation(0, 0);
        MapLocation goal = new MapLocation(10,10);
        System.out.println(PATH_FINDER(start, goal));
    }

    /**
     * This is the algorithm that will compute the path between the current location and the goal location
     * @param start
     * The location at which the robot begins its motion
     * @param goal
     * The location to which the robot travels
     * @return
     * A string representing the sequecnce of coordinate points that the robot passes through.
     */
    public static String PATH_FINDER(MapLocation start, MapLocation goal){
        Queue<MapLocation> neighbors = new LinkedList<>();              // Declares a queue corresponding to neighboring MapLocations
        neighbors.add(start);                                   // Add the initial MapLocation to the queue
        double f_cost = CALCULATE_H_COST(start, goal);          // f_cost stores the computed cost of the path passing through the starting point
        double g_cost = 0;                                      // Stores the cost of traveling to a neighboring MapLocation
        MapLocation current = start;                            // Set the current location to start

        while(!neighbors.peek().equals(null)){                         // While the neighbors queue is not empty (empty only when reaches goal)
            neighbors.remove();
            MapLocation N = new MapLocation(current.getX(), current.getY()+1);
            MapLocation E = new MapLocation(current.getX()+1, current.getY());
            MapLocation S = new MapLocation(current.getX(), current.getY()-1);
            MapLocation W = new MapLocation(current.getX()-1, current.getY());              // These are all of the 8 neighbors by compass direction
            MapLocation NE = new MapLocation(current.getX()+1, current.getY()+1);
            MapLocation NW = new MapLocation(current.getX()-1, current.getY()+1);
            MapLocation SE = new MapLocation(current.getX()+1, current.getY()-1);
            MapLocation SW = new MapLocation(current.getX()-1, current.getY()-1);


            neighbors.add(N);
            neighbors.add(E);
            neighbors.add(S);
            neighbors.add(W);                               // Add neighbors to the neighbors queue
            neighbors.add(NE);
            neighbors.add(NW);
            neighbors.add(SE);
            neighbors.add(SW);

            f_cost = CALCULATE_H_COST(neighbors.peek(), goal);                // Estimates the cost of traveling to the goal from the current cell

            for(int i = 0; i < 8; i++){                                 // For each neighbor
                if(CALCULATE_H_COST(neighbors.peek(), goal) <= f_cost) {       // if the cost of traveling from that neighbor to the goal is lower than the current cost of traveling to the goal,
                    f_cost = CALCULATE_H_COST(neighbors.peek(), goal);        // reset the current cost,
                    current = neighbors.peek();                         // set current to the cheapest neighbor
                }
                if(f_cost == Double.POSITIVE_INFINITY)              // If the cost is infinite
                    return "Path not found.";

                else if(current.equals(goal))                       // If the goal is reached
                    return CONSTRUCT_PATH(goal);                    // Run a recursive method to establish the path

                double g_cost_temp = g_cost + CALCULATE_COST(current, neighbors.peek());        // Call to another method
                double f_cost_temp = g_cost_temp + CALCULATE_H_COST(neighbors.peek(), goal);    // Call to another method
                if(f_cost_temp < f_cost){
                    try{
                        neighbors.peek().setPrevious(current);
                    }
                    catch(CloneNotSupportedException cnse){
                        System.out.println("The MapLocation object cannot be cloned.");
                    }
                    g_cost = g_cost_temp;                   // Update cost values
                    f_cost = f_cost_temp;
                }

                neighbors.remove();                                     // Remove each neighbor from the neighbors queue when processed
            }                                       // Exit for-each-neighbor loop

            if(!current.equals(goal))                   // If the goal has not been reached,
                ((LinkedList<MapLocation>) neighbors).add(current);         // Add current to the linked list so as not to violate the while loop condition
        }                                          // Exit while-not-reached-goal loop

        return "Path not found.";
    }

    /**
     * Recursive method
     * @param current
     * The current MapLocation
     * @return
     * String describing the path of the rover by coordinates
     */
    public static String CONSTRUCT_PATH(MapLocation current){
        if (!current.getPrevious().equals(null)){                       // If current has a previous node (not the initial location)
            String p = CONSTRUCT_PATH(current.getPrevious());           // recursively call on that previous node
            return p + ", " + current.toString();                       // Return the recursive String
        }
        else
            return current.toString();                                  // Stopping condition when initial location is reached
    }

    /**
     * @param i
     * A MapLocation
     * @param j
     * Another MapLocation
     * @return
     * The cost of traveling from i to j
     */
    public static double CALCULATE_COST(MapLocation i, MapLocation j){
        double MASS = 50.0;                             // Approximate mass of rover
        double K_FRICTION = 0.2;                        // Approximates coefficient of kinetic friction
        double S_FRICTION = 0.3;                        // Approximates coefficient of static friction
        double P_MAX = 10;                              // Approximates the maximum available motion power of the rover
        double V = 0.5;                                 // Approximates the velocity of the robot (assumed to be constant in this model)
        double G = 9.81;                                // Stores the gravitational constant

        double z = test_function(j.getX(), j.getY()) - test_function(i.getX(), i.getY());       // Simulated difference in height

        // Calculate the angle of inclination
        double ang_inclination = Math.atan(z/Math.sqrt(Math.pow(j.getX() - i.getX(),2) + Math.pow(j.getY() - i.getY(),2)));

        // Calculate the critical impermissible angle for uphill traversal
        double critical_angle = Math.min(Math.asin(P_MAX/(V*MASS*G*Math.sqrt(Math.pow(K_FRICTION, 2)+1))) - Math.atan(K_FRICTION) , Math.atan(S_FRICTION - K_FRICTION));

        // Calculate the critical breaking angle for downhill traversal
        double critical_breaking = -Math.atan(K_FRICTION);

        if(ang_inclination > critical_angle)
            return Double.POSITIVE_INFINITY;
        else if(ang_inclination <= critical_angle && ang_inclination > critical_breaking){
            double energy_cost = MASS*G*Math.sqrt(Math.pow(j.getX() - i.getX(), 2) + Math.pow(j.getY() - i.getY(), 2) + Math.pow(z, 2))*(K_FRICTION*Math.cos(ang_inclination) + Math.sin(ang_inclination));
            return energy_cost;
        }
        else
            return 0;
    }

    /**
     * @param now
     * Current MapLocation
     * @param goal
     * Goal MapLocation
     * @return
     * The approximate energy cost of traveling from the current MapLocation to the goal
     */
    public static double CALCULATE_H_COST(MapLocation now, MapLocation goal){
        double MASS = 50.0;                             // Approximate mass of rover
        double G = 9.81;                                // Stores the gravitational constant
        double K_FRICTION = 0.2;                        // Approximates coefficient of kinetic friction
        double S_FRICTION = 0.3;                        // Approximates coefficient of static friction
        double P_MAX = 10;                              // Approximates the maximum available motion power of the rover
        double V = 0.5;                                 // Approximates the velocity of the robot (assumed to be constant in this model)


        // Calculate the critical impermissible angle for uphill traversal
        double critical_angle = Math.min(Math.asin(P_MAX/(V*MASS*G*Math.sqrt(Math.pow(K_FRICTION, 2)))) - Math.atan(K_FRICTION) , Math.atan(S_FRICTION - K_FRICTION));


        double energy = CALCULATE_COST(now, goal);
        if(energy != Double.POSITIVE_INFINITY)
            return energy;
        else{
            energy = MASS*G*(K_FRICTION*Math.cos(critical_angle) + Math.sin(critical_angle))*(test_function(goal.getX(), goal.getY()) - test_function(now.getX(), now.getY()))/Math.sin(critical_angle);
            return energy;
        }
    }

    /**
     * Function that is used to simulate the surface for test of the algorithim
     * @param x
     * x-coordinate of a MapLocation
     * @param y
     * y-coordinate of a MapLocation
     * @return
     * The elevation of the surface at a MapLocation with coordinates (x, y)
     */
    public static double test_function(int x, int y){
        return (double)x+y;
    }
}
