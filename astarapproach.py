class AStarSolver:
    def __init__(self, start_state, goal_state, heuristic='manhattan'):
        self.start_state = start_state
        self.goal_state = goal_state
        self.heuristic_choice = heuristic
        self.open_list = []  # List to hold nodes to be evaluated (priority queue)
        self.closed_list = set()  # Set to hold already evaluated nodes
        self.came_from = {}  # To reconstruct the path
        self.g_cost = {}  # Actual cost from start to each node
        self.f_cost = {}  # Estimated cost from start to goal through each node

    def heuristic(self, state):
        """
        Heuristic function to estimate the cost from the current state to the goal state.
        Can choose between 'manhattan' or 'hamming'.
        """
        if self.heuristic_choice == 'manhattan':
            return self.manhattan_distance(state)
        elif self.heuristic_choice == 'hamming':
            return self.hamming_distance(state)

    def manhattan_distance(self, state):
        """
        Manhattan distance heuristic: sum of the vertical and horizontal distances 
        of the tiles from their goal positions.
        """
        distance = 0
        for i in range(3):
            for j in range(3):
                tile = state[i][j]
                if tile != 0:
                    goal_x, goal_y = divmod(tile - 1, 3)
                    distance += abs(goal_x - i) + abs(goal_y - j)
        return distance

    def hamming_distance(self, state):
        """
        Hamming distance heuristic: counts how many tiles are out of place compared to the goal state.
        """
        distance = 0
        for i in range(3):
            for j in range(3):
                if state[i][j] != 0 and state[i][j] != self.goal_state[i][j]:
                    distance += 1
        return distance

    def get_neighbors(self, state):
        """
        Generate possible new states by moving the blank tile (0) in the four possible directions.
        """
        neighbors = []
        blank_x, blank_y = self.find_blank_tile(state)
        moves = [("up", (-1, 0)), ("down", (1, 0)), ("left", (0, -1)), ("right", (0, 1))]

        for move_name, (dx, dy) in moves:
            new_x, new_y = blank_x + dx, blank_y + dy
            if 0 <= new_x < 3 and 0 <= new_y < 3:
                # Create new state by swapping the blank with the adjacent tile
                new_state = [row[:] for row in state]
                new_state[blank_x][blank_y], new_state[new_x][new_y] = new_state[new_x][new_y], new_state[blank_x][blank_y]
                neighbors.append(new_state)
        return neighbors

    def find_blank_tile(self, state):
        """
        Find the position of the blank tile (0) in the current state.
        """
        for i in range(3):
            for j in range(3):
                if state[i][j] == 0:
                    return i, j

    def reconstruct_path(self, current):
        """
        Reconstruct the path from the goal to the start by following the came_from map.
        """
        path = []
        while current in self.came_from:
            path.append(current)
            current = self.came_from[current]
        path.reverse()
        return path

    def a_star_search(self):
        """
        The A* search algorithm to solve the 8-puzzle problem.
        """
        start_tuple = tuple(tuple(row) for row in self.start_state)  # Convert to hashable form
        goal_tuple = tuple(tuple(row) for row in self.goal_state)

        # Initialize the starting node
        self.g_cost[start_tuple] = 0
        self.f_cost[start_tuple] = self.heuristic(self.start_state)
        self.open_list.append((self.f_cost[start_tuple], self.start_state))

        while self.open_list:
            # Get the node with the lowest f_cost
            self.open_list.sort()  # Sort by f_cost to simulate a priority queue
            _, current_state = self.open_list.pop(0)  # Get the state with the lowest f_cost

            current_tuple = tuple(tuple(row) for row in current_state)
            if current_tuple == goal_tuple:
                return self.reconstruct_path(current_tuple)

            self.closed_list.add(current_tuple)

            # Generate neighbors and evaluate them
            for neighbor in self.get_neighbors(current_state):
                neighbor_tuple = tuple(tuple(row) for row in neighbor)

                if neighbor_tuple in self.closed_list:
                    continue

                tentative_g_cost = self.g_cost[current_tuple] + 1  # Each move has a cost of 1

                if neighbor_tuple not in [state for _, state in self.open_list]:
                    self.open_list.append((float('inf'), neighbor))  # Add to open list with a large f_cost

                if tentative_g_cost >= self.g_cost.get(neighbor_tuple, float('inf')):
                    continue  # Not a better path

                # This path is the best so far
                self.came_from[neighbor_tuple] = current_tuple
                self.g_cost[neighbor_tuple] = tentative_g_cost
                self.f_cost[neighbor_tuple] = tentative_g_cost + self.heuristic(neighbor)

                # Update open_list with new f_cost
                for i, (f_cost, state) in enumerate(self.open_list):
                    if state == neighbor:
                        self.open_list[i] = (self.f_cost[neighbor_tuple], neighbor)
                        break

        return None  # No solution found

# Example usage
start_state = [[1, 2, 3], [4, 0, 6], [7, 5, 8]]  # Initial state of the puzzle
goal_state = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]  # Goal state

solver = AStarSolver(start_state, goal_state, heuristic='manhattan')
solution_path = solver.a_star_search()

if solution_path:
    print("Solution found!")
    for step in solution_path:
        for row in step:
            print(row)
        print("----")
else:
    print("No solution found.")
