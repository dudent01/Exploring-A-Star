// Denis Rozhnov
// SBRT Software Team: Autonavigation

// This class represents the location of the rover on the Cartesian plane, regardless of elevation.
// We assume that the elevation is always 0, since we compute the adjacent elevations relative to the current one
// This class associates any position with the position of neighboring locations.
// This is useful for finding energy-efficient paths on uneven terrains

public class MapLocation implements Cloneable{
    private int x;                  // These are the x and y coordinates of the location on the map
    private int y;                  // They are a distinct pair for each distinct location, in order to specity the goal

    /**
     * Default constructor sets the coordinates of the MapLocation to (0, 0)
     */
    public MapLocation(){
        x = 0;
        y = 0;
    }

    /**
     * Initializes the x and y coordinates of a MapLocation to specific values
     * @param xc
     * The value to which the x-coordinate is initialized
     * @param yc
     * The value to which the y-coordinate is initialized
     */
    public MapLocation(int xc, int yc){
        x = xc;
        y = yc;
    }

    /**
     * @return
     * The value of the x-coordinate
     */
    public int getX(){
        return x;
    }

    /**
     * @return
     * The value of the y-coordinate
     */
    public int getY(){
        return y;
    }

    MapLocation previous;                                   // Stores the previous MapLocation along the route

    public void setPrevious(MapLocation m) throws CloneNotSupportedException{
        previous = (MapLocation)m.clone();
    }

    /**
     * @return
     * The previous MapLocation
     */
    public MapLocation getPrevious(){
        return previous;
    }
    /**
     * @return
     * String representation of the MapLocation as the coordinate pair
     * Needed when returning the output of the algorithm
     */
    public String toString(){
        String s = "(" + x + ", " + y + ")";
        return s;
    }
}
