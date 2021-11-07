import heapq

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSEDD list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the neighbour
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._h = 0
        self._cost = 0
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the neighbour in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSEDD list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def get_h(self):
        """
        Returns the h-value of the state
        """
        return self._h
    
    def get_cost(self):
        """
        Returns the cost of the state (g for Dijkstra's and f for A*)
        """
        return self._cost
    
    def set_g(self, cost):
        """
        Sets the g-value of the state
        """
        self._g = cost
    
    def set_h(self, h):
        """
        Sets the h-value of the state
        """
        self._h = h
    
    def set_cost(self, cost):
        """
        Sets the cost of a state (g for Dijkstra's and f for A*)
        """
        self._cost = cost

class Search:
    """
    Interface for a search algorithm. It contains an OPEN list and a CLOSEDD list.

    The OPEN list is implemented with a heap, which can be done with the library heapq
    (https://docs.python.org/3/library/heapq.html).    
    
    The CLOSEDD list is implemented as a dictionary where the state hash value is used as key.
    """
    def __init__(self, gridded_map):
        self.map = gridded_map
        self.OPEN = []
        self.CLOSED = {}
    
    def search(self, start, goal):
        """
        Search method that needs to be implemented (either Dijkstra or A*).
        """
        raise NotImplementedError()
            
class Dijkstra(Search):
    
    
    def search(self, start, goal):
        """
        Disjkstra's Algorithm: receives a start state and a goal state as input. It returns the
        cost of a path between start and goal and the number of neighbour expanded.

        If a solution isn't found, it returns -1 for the cost.
        """
        self.OPEN = []
        self.CLOSED = {}        
        #push the start state into the OPEN
        heapq.heappush(self.OPEN,start)
        #set the cost to 0 for start node
        start.set_cost(0)
        heapq.heapify(self.OPEN)
        #add the start state into the CLOSED 
        self.CLOSED[start.state_hash()] = start
        node_expanded = 0
        while self.OPEN:
            #pop the smallest item, start node
            node = heapq.heappop(self.OPEN)
            node_expanded += 1            
            #if node is the goal
            if node == goal:
                #return the cost and the number of node expanded
                return self.CLOSED[node.state_hash()].get_cost(), node_expanded
            
            #get all the neighbours
            neighbours = self.map.successors(node)
            
            for neighbour in neighbours:
                #if the neighbour has not been reached yet
                if neighbour.state_hash() not in self.CLOSED:
                    #push the neighbour into OPEN
                    heapq.heappush(self.OPEN,neighbour)    
                    #set the cost of this state
                    neighbour.set_cost(neighbour.get_g())
                    heapq.heapify(self.OPEN)
                    #add the node into CLOSED
                    self.CLOSED[neighbour.state_hash()] = neighbour
                #if the neighbour has been reached before and the newer cost of this neighbour is smaller than the on
                #stored in the CLOSED
                if ((neighbour.state_hash() in self.CLOSED) & (neighbour.get_g() < self.CLOSED[neighbour.state_hash()].get_cost())):
                    #update the new cost
                    neighbour.set_cost(neighbour.get_g())
                    heapq.heapify(self.OPEN)
                    #update the CLOSED
                    self.CLOSED[neighbour.state_hash()] = neighbour

                    
        return -1, 0
    
class AStar(Search):
    
    def h_value(self, state):
        x_distance = abs(state.get_x() - self.goal.get_x())
        y_distance = abs(state.get_y() - self.goal.get_y())
        return (max(x_distance,y_distance) + 0.5*min(x_distance,y_distance))
            
    def search(self, start, goal):
        """
        A* Algorithm: receives a start state and a goal state as input. It returns the
        cost of a path between start and goal and the number of neighbour expanded.

        If a solution isn't found, it returns -1 for the cost.
        """
        self.OPEN = []
        self.CLOSED = {} 
        self.goal = goal
        node_expanded = 0
        #push the start node
        heapq.heappush(self.OPEN, start)
        while self.OPEN:
            #pop the smallest item
            node = heapq.heappop(self.OPEN)
            node_expanded += 1
            #if found goal
            if node == self.goal:
                return node.get_cost(), node_expanded
            self.CLOSED[node.state_hash()] = node 
            neighbours = self.map.successors(node)
            for neighbour in neighbours:
                #if neighbour has been in CLOSED
                if neighbour.state_hash() in self.CLOSED:
                    continue
                if neighbour in self.OPEN:
                    #get the neighbour in OPEN that has been stored before,(it already in OPEN)
                    neighbour_in_open = self.OPEN[self.OPEN.index(neighbour)]
                    if neighbour.get_g() < neighbour_in_open.get_g():
                        #set the new g value
                        neighbour_in_open.set_g(neighbour.get_g())
                        #set the new cost
                        neighbour_in_open.set_cost(neighbour.get_g()+self.h_value(neighbour))
                        heapq.heapify(self.OPEN)
                else:
                    #if not reached yet, push the node with f score
                    neighbour.set_cost(neighbour.get_g()+self.h_value(neighbour))
                    heapq.heappush(self.OPEN, neighbour)
        
        return -1, 0
