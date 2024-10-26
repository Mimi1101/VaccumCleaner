import heapq

"For my implementation, to calculate h1 and h2, head to the main method in line 316, for now the default calc"
"h1(n) = minimum Manhattan distance to a dirty square + 2 * number of dirty squares."
"h2(n) = maximum Manhattan distance to any dirty square + 2 * number of dirty squares."

""

class State:
    """
    Represents a state in the grid for the cleaning agent.

    Attributes:
        agent_position: The current position of the agent as (x, y).
        dirty_squares: representing dirty squares.
        g_value : cumulative path cost from the start state to this node.
        h_value : heuristic value estimated from this node to the goal.
        fn_value : The total estimated cost (f(n) = g(n) + h(n)).
        action : The action taken to reach this state.
        parent: The parent node from which this state was generated.
        heuristic: The heuristic function to use ('h1' or 'h2').
    """
    
    def __init__(self, agent_position, dirty_squares, heuristic='h1'): # default heuristic function is h1
        self.agent_position = agent_position
        self.dirty_squares = set(dirty_squares)
        self.g_value = 0
        self.h_value = 0
        self.fn_value = 0
        self.action = None
        self.parent = None
        self.heuristic = heuristic

    def __lt__(self, other):
        """
        Defines the less-than comparison based on f(n) for priority queue ordering.
        """
        return self.fn_value < other.fn_value

    def __hash__(self):
        """
        making the objects hashable
        """
        return hash((self.agent_position, frozenset(self.dirty_squares)))

    def __eq__(self, other):
        """
        Checks equality based on agent position and dirty squares.
        """
        return (self.agent_position == other.agent_position and
                self.dirty_squares == other.dirty_squares)

    def goal_state(self):
        """
        to check if we reached the goal state that is all dirty squares are clean
        """
        return len(self.dirty_squares) == 0

    def calculate_manhattan_distance(self, a, b):
        """
        Calculates the Manhattan distance between two points.
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def calculate_heuristic_one_value(self):
        """
        Calculates h1(n) = minimum Manhattan distance to a dirty square + 2 * number of dirty squares.
        """
        if not self.dirty_squares:
            self.h_value = 0
            return self.h_value
        min_distance = min(
            self.calculate_manhattan_distance(self.agent_position, dirty_square)
            for dirty_square in self.dirty_squares
        )
        dirty_squares_cost = 2 * len(self.dirty_squares)
        self.h_value = min_distance + dirty_squares_cost
        return self.h_value

    def calculate_heuristic_two_value(self):
        """
        Calculates h2(n) = maximum Manhattan distance to any dirty square + 2 * number of dirty squares.
        """
        if not self.dirty_squares:
            self.h_value = 0
            return self.h_value

        max_distance = max(
            self.calculate_manhattan_distance(self.agent_position, dirty_square)
            for dirty_square in self.dirty_squares
        )
        dirty_squares_cost = 2 * len(self.dirty_squares)
        self.h_value = max_distance + dirty_squares_cost
        return self.h_value

    def calculate_heuristic_value(self):
        """
        Guides which heursitic to choose, h1 or h2
        """
        if self.heuristic == 'h1':
            return self.calculate_heuristic_one_value()
        elif self.heuristic == 'h2':
            return self.calculate_heuristic_two_value()
        else:
            raise ValueError("Unknown heuristic. Choose 'h1' or 'h2'.")

    def calculate_path_cost(self):
        """
        Calculates g(n)
        """
        if self.parent is None:
            self.g_value = 0  
        else:
            action_cost = 1  
            self.g_value = self.parent.g_value + action_cost
            remaining_dirty_squares = len(self.dirty_squares)
            self.g_value += 2 * remaining_dirty_squares

        return self.g_value

    def calculate_fn_value(self):
        """
        Calculates the total estimated cost f(n) = g(n) + h(n).
        """
        self.fn_value = self.g_value + self.h_value
        return self.fn_value

    def apply_actions(self, grid_size=5):
        """
        Generates all possible successor states from the current node.

        Possible actions:
            - Move Left, Right, Up, Down within grid boundaries.
            - 'Suck' to clean the current square if it's dirty.
        """
        successors = []
        x, y = self.agent_position
        possible_actions = []

        # Determine possible movement actions based on current position
        if x > 1:
            possible_actions.append('Left')
        if x < grid_size:
            possible_actions.append('Right')
        if y > 1:
            possible_actions.append('Down')
        if y < grid_size:
            possible_actions.append('Up')

        possible_actions.append('Suck')

        for action in possible_actions:
            if action == 'Suck':
                # Handle 'Suck' action only if the current square is dirty
                if (x, y) in self.dirty_squares:
                    new_dirty_squares = self.dirty_squares.copy()
                    new_dirty_squares.remove((x, y))  # Clean the current square
                    new_state = State((x, y), new_dirty_squares, heuristic=self.heuristic)
                    new_state.parent = self
                    new_state.action = action
                    new_state.calculate_path_cost()
                    new_state.calculate_heuristic_value()
                    new_state.calculate_fn_value()
                    successors.append(new_state)
                else:
                    continue  # Skip 'Suck' if the square is already clean
            else:
                # Handle movement actions
                new_x, new_y = x, y
                if action == 'Left':
                    new_x -= 1
                elif action == 'Right':
                    new_x += 1
                elif action == 'Down':
                    new_y -= 1
                elif action == 'Up':
                    new_y += 1

                new_dirty_squares = self.dirty_squares.copy()
                new_state = State((new_x, new_y), new_dirty_squares, heuristic=self.heuristic)
                new_state.parent = self
                new_state.action = action
                new_state.calculate_path_cost()
                new_state.calculate_heuristic_value()
                new_state.calculate_fn_value()
                successors.append(new_state)

        return successors


class AStar:
    """
    Implements the A* search algorithm 

    Attributes:
        open_list: A priority queue (min-heap) of states to be expanded.
        closed_set : A set of nodes that have already been visited.
        heuristic (str): The heuristic function to use ('h1' or 'h2').
    """

    def __init__(self, heuristic='h1'):
        """
        Initializes the AStar search with a specified heuristic.

        Args:
            heuristic (str): The heuristic function to use ('h1' or 'h2').
        """
        self.expand_list  = []  # Priority queue for states to be expanded
        self.closed_set = set()  # Explored states
        self.heuristic = heuristic

    def add_to_open_list(self, state):
        """
        Adds a state to the expand list (priority queue).

        Args:
            state (State): The state to add.
        """
        heapq.heappush(self.expand_list, state)

    def pop_from_open_list(self):
        """
        Pops the state with the lowest f(n) from the open list.

        Returns:
            State: The state with the lowest f(n) value.
        """
        return heapq.heappop(self.expand_list)

    def search(self, initial_state):
        """
        Performs the A* search algorithm.
        """
        self.add_to_open_list(initial_state)
        nodes_expanded = 0

        while self.expand_list:
            current_state = self.pop_from_open_list()

            # Skip if the state has already been explored
            if current_state in self.closed_set:
                continue

            # Mark the current state as explored
            self.closed_set.add(current_state)
            nodes_expanded += 1

            # Check if the goal state is reached
            if current_state.goal_state():
                return current_state, nodes_expanded

            # Expand the current state by generating successors
            successors = current_state.apply_actions()
            for successor in successors:
                if successor not in self.closed_set:
                    self.add_to_open_list(successor)

        # Return None if no solution is found
        return None, nodes_expanded

    def reconstruct_path(self, goal_state):
        """
        Reconstructs the path from the initial state to the goal state.
        """    
        path = []
        fn_values = []
        g_values = []
        states = []
        h_values = []
        state = goal_state

        # Traverse from the goal state back to the initial state
        while state is not None:
            path.append(state.action)
            fn_values.append(state.fn_value)
            g_values.append(state.g_value)
            h_values.append(state.h_value)
            states.append(state)
            state = state.parent

        # Reverse the lists to get the path from start to goal
        path.reverse()
        fn_values.reverse()
        g_values.reverse()
        h_values.reverse()
        states.reverse()

        # Exclude the initial 'None' action
        return path[1:], fn_values, g_values, h_values, states

    def display_grid(self, agent_position, dirty_squares, fn_value, g_value, h_value, grid_size=5):
        """
        Displays the current grid state with the agent and dirty squares.
        """
        for y in range(grid_size, 0, -1):
            row = ''
            for x in range(1, grid_size + 1):
                if (x, y) == agent_position:
                    row += 'A '  # Agent's position
                elif (x, y) in dirty_squares:
                    row += 'D '  # Dirty square
                else:
                    row += 'C '  # Clean square
            print(row)
        print(f"g(n) = {g_value}, h(n) = {h_value}, f(n) = {fn_value}\n")


def main():

    grid_size = 5  
    initial_agent_pos = (1, 1)  
    # Define dirty squares (all squares in the top row are dirty)
    dirty_squares = [(x, grid_size) for x in range(1, grid_size + 1)]

    # Part (A): Using heuristic h1(n)
    chosen_heuristic = 'h1'  # to calculate just change to h2

    # Initialize the initial state
    initial_state = State(initial_agent_pos, dirty_squares, heuristic=chosen_heuristic)
    initial_state.calculate_path_cost()
    initial_state.calculate_heuristic_value()
    initial_state.calculate_fn_value()

    # Display initial state information
    print(f"Initial State: Agent at {initial_state.agent_position}, "
          f"Dirty Squares: {sorted(initial_state.dirty_squares)}")
    print(f"Initial f(n) = {initial_state.fn_value} (g(n) = {initial_state.g_value}, "
          f"h(n) = {initial_state.h_value})\n")

    # Initialize the A* search with the chosen heuristic
    astar = AStar(heuristic=chosen_heuristic)
    astar.add_to_open_list(initial_state)
    print("Starting A* Search...\n")

    # Perform the A* search
    goal_state, nodes_expanded = astar.search(initial_state)

    # Check if a goal state was found
    if goal_state:
        # Reconstruct the path from initial state to goal state
        path, fn_values, g_values, h_values, states = astar.reconstruct_path(goal_state)
        print("Sequence of actions and grids along the optimal path:\n")

        for i, (action, fn, g, h, state) in enumerate(zip(path, fn_values, g_values, h_values, states)):
            if i == 0:
                # Display initial state
                print(f"Initial State: g(n) = {g}, h(n) = {h}, f(n) = {fn}")
                astar.display_grid(state.agent_position, state.dirty_squares, fn, g, h)
            else:
                # Display each step in the path
                print(f"Step {i}: Action = {action}, g(n) = {g}, h(n) = {h}, f(n) = {fn}")
                astar.display_grid(state.agent_position, state.dirty_squares, fn, g, h)

        print(f"Total number of nodes expanded: {nodes_expanded}")
    else:
        print("No solution found.")


if __name__ == '__main__':
    main()
