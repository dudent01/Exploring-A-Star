# This class defines a node representing a location on the cartesian plane.

class MapNode:
    # Parametric constructor
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Getter method for x
    def get_x(self):
        return self.x

    # Getter method for y
    def get_y(self):
        return self.y

    # Setter method for previous location
    def set_previous(self, prev):
        self.previous = prev

    # Getter method for previous location
    def get_previous(self):
        return self.previous

    # toString method of the MapNode class
    def to_string(self):
        return "({0}, {1})".format(self.x, self.y)