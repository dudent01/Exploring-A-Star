// Denis Rozhnov
// SBRT Software Team: Autonavigation

// This class is a queue for storing the neighbors of the current MapLocation
import java.util.LinkedList;

public class MapNeighbors extends LinkedList<MapLocation>{
    LinkedList<MapLocation> neighbors;                          // The class will declare functions that will manipulate on neighbors

    /**
     * Default constructor for the  MapNeighbors class
     */
    public MapNeighbors(){
        neighbors = new LinkedList<MapLocation>();              // neighbors MapNeighbors queue is initiated
    }

    /**
     * Enqueue method
     * @param m
     * MapLocation object that is enqueued to the queue
     */
    public void enqueue(MapLocation m){
        neighbors.add(m);
    }

    public MapLocation dequeue(){
        return (MapLocation) neighbors.removeFirst();
    }
}

