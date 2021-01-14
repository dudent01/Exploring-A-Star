# This class describes the location of the rover on the terrain on the Cartesian plane

class MapLocation:

    # Parametric constructor
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Getter method for x
    def get_x(self):
        return self.x

    #Getter method for y
    def get_y(self):
        return self.y

    # Setter method for previous location
    def set_previous(self, prev):
        self.previous = prev


    def get_previous(self):
        return self.previous

    # toString method of the MapLocation class
    def to_string(self):
        return "({0}, {1})".format(self.x, self.y)

