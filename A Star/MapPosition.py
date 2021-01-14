# This class represents a location in the grid

class MapPosition:
    # Parametric constructor
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.walkable = True
        self.previous = None
        self.g_cost = 0
        self.h_cost = 0
        self.change = False                     # Variable that indicates that the MapPosition has been a point at which the path changed direction

    # Equality of two MapPositions
    def __eq__(self, other):
        if (self.x == other.x and self.y == other.y):
            return True
        return False

    def __lt__(self, other):
        if (self.f_cost() < other.f_cost() or self.f_cost() == other.f_cost() and self.get_h_cost() < other.get_h_cost()):
            return self
        return other

    # Getter method for x
    def get_x(self):
        return self.x

    # Setter method for x
    def set_x(self, val):
        self.x = val

    # Getter method for y
    def get_y(self):
        return self.y

    # Setter method for y
    def set_y(self, val):
        self.y = val

    # Setter method for previous location
    def set_previous(self, prev):
        self.previous = prev

    # Getter method for previous location
    def get_previous(self):
        return self.previous

    # Setter method for walkability
    def set_walkable(self, boolean):
        self.walkable = boolean

    # Getter method for walkability
    def get_walkable(self):
        return self.walkable

    # Setter method for the g cost, the cost of getting from start to this node
    def set_g_cost(self, cost_to):
        self.g_cost = cost_to

    # Getter method for the g cost
    def get_g_cost(self):
        return self.g_cost

    # Setter method for the h cost, the heuristic cost of getting from this node to goal
    def set_h_cost(self, cost_from):
        self.h_cost = cost_from

    # Getter method for the h cost
    def get_h_cost(self):
        return self.h_cost

    # Computes the f cost, the total cost of travelling through this node
    def f_cost(self):
        return self.g_cost + self.h_cost

    # Setter method for the change variable
    def set_change(self, turn):
        self.change = turn

    # Getter method for the change variable
    def get_change(self):
        return self.change

    # toString method of the MapNode class
    def to_string(self):
        return "({0}, {1})".format(str(self.x), str(self.y))