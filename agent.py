
import heapq






class State:
    def __init__(self, agent_position, dirty_squares):
        self.agent_position = agent_position
        self.dirty_squares = dirty_squares
        self.g_value = 0
        self.h1_value = 0
        self.h2_value = 0
        self.fn_value = 0
        self.actions = None
        self.parent = None

    
    def __lt__ (self, option):
        """
        Heapq will compare states based on the f(n) value and will return the minimum f(n) value
        """
        return self.fn_value < option.fn_value
    
    def __hash__(self):
        """
        This hash stores in sets and checks uniqueness.
        """
        return hash((self.agent_position, tuple(self.dirty_squares)))
    
    def __eq__(self, other):
        """
        checking the equality to compare between the states.
        """
        return self.agent_position == other.agent_position and self.dirty_squares == other.dirty_squares
    
    def goal_state(self):
        """
        checks to see if we reached the goal state that is no more dirty squares left
        """
        return len(self.dirty_squares) == 0
    
    def calculate_manhattan_distance(self, x, y):
        """
        Calculates the Manhattan distance between two points (x1, y1) and (x2, y2).
        """
        manhattan_distance = abs(x[0] - y[0]) + abs(x[1] - y[1])
        return manhattan_distance

    def calculate_heuristic_one_value(self):
        """
        Calculates h1(n) = minimum of the manhattan distance + number of the dirty squares left
        """

        min_distance = min(self.calculate_manhattan_distance(self.agent_position, dirty_square)
                           for dirty_square in self.dirty_squares)
        self.h1_value = min_distance + len(self.dirty_squares)
        return self.h1_value

    def calculate_path_cost(self):
        """
        Calculating g(n)
        """
        action_cost =1
        dirty_square_cost = len(self.dirty_squares) * 2
        self.g_value = action_cost + dirty_square_cost
        return self.g_value
    
    def calculate_fn_value(self):
        """
        Calculating f(n) = g(n) + h1(n)
        """
        self.fn_value = self.g_value + self.h1_value
        return self.fn_value

    def apply_actions(self, grid_size = 5):
        successors = []
        x,y= self.agent_position
        possible_actions = []

        #checking for possible movements
        if x>1:
            possible_actions.append('Left')
        if x <= grid_size:
            possible_actions.append('Right')
        if y > 1:
            possible_actions.append('Down')
        if y <= grid_size:
            possible_actions.append('Up')
        possible_actions.append('Suck')

        for action in possible_actions:
            new_x, new_y = x, y
            new_dirty_squares = self.dirty_squares.copy()
            if action == 'Left':
                new_x -= 1
            elif action == 'Right':
                new_x += 1
            elif action == 'Down':
                new_y -= 1
            elif action == 'Up':
                new_y += 1
            elif action == 'Suck':
                if(x,y) in new_dirty_squares:
                    new_dirty_squares.remove((x,y))

            new_state = State((new_x, new_y), new_dirty_squares)
            new_state.parent = self
            new_state.actions = action
            new_state.g_value = new_state.calculate_path_cost()
            new_state.calculate_heuristic_one_value()
            new_state.calculate_fn_value()
            successors.append(new_state)
        return successors


        
    
   

class Astar():

    def __init__(self):
        self.expand_list = [] # priority queue (min heap) for the states to be expanded       
        self.visited_list = set() # set for tracking visited states

    def add_to_expand_list(self, state):
        """
        Adding the state to the expand list
        """
        heapq.heappush(self.expand_list, state)

    def remove_from_expand_list(self, state):
        """
        Removing the state with the lowest f(n) value 
        """
        return heapq.heappop(self.expand_list)

    def already_visited(self, state):
        """
        checks if a state is already visited by checking if its in the visited list
        """
        return state in self.visited_list

    def add_to_visited_list(self, state):
        """
        Adds a state to the visited list
        """
        self.visited_list.add(state)

 


        




if __name__ == '__main__':
    grid_size = 5
    initial_agent_pos = (1,1)



           
    
    


    
    
        


