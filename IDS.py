class IterativeDeepeningSolver:
    def __init__(self, start_state, goal_state):
        self.start_state = start_state
        self.goal_state = goal_state
        self.moves = [("up", (-1, 0)), ("down", (1, 0)), ("left", (0, -1)), ("right", (0, 1))]  # Possible moves

    def find_blank_tile(self, state):
        """
        Find the position of the blank tile (0) in the current state.
        """
        for i in range(3):
            for j in range(3):
                if state[i][j] == 0:
                    return i, j

    def get_neighbors(self, state):
        """
        Generate possible new states by moving the blank tile (0) in the four possible directions.
        """
        neighbors = []
        blank_x, blank_y = self.find_blank_tile(state)

        for move_name, (dx, dy) in self.moves:
            new_x, new_y = blank_x + dx, blank_y + dy
            if 0 <= new_x < 3 and 0 <= new_y < 3:
                # Create new state by swapping the blank with the adjacent tile
                new_state = [row[:] for row in state]
                new_state[blank_x][blank_y], new_state[new_x][new_y] = new_state[new_x][new_y], new_state[blank_x][blank_y]
                neighbors.append(new_state)
        return neighbors

    def depth_limited_search(self, state, depth, visited):
        """
        Depth-Limited Search (DFS with a depth limit).
        """
        if state == self.goal_state:
            return [state]
        
        if depth == 0:
            return None

        visited.add(tuple(tuple(row) for row in state))  # Convert state to tuple to hash it

        for neighbor in self.get_neighbors(state):
            neighbor_tuple = tuple(tuple(row) for row in neighbor)
            if neighbor_tuple not in visited:
                result = self.depth_limited_search(neighbor, depth - 1, visited)
                if result is not None:
                    return [state] + result  # Prepend current state to path

        visited.remove(tuple(tuple(row) for row in state))  # Remove from visited set
        return None

    def iterative_deepening_search(self):
        """
        Iterative Deepening Search (IDS) for solving the 8-puzzle problem.
        """
        depth = 0
        while True:
            visited = set()
            result = self.depth_limited_search(self.start_state, depth, visited)
            if result is not None:
                return result
            depth += 1


# Example usage:
start_state = [[1, 2, 3], [4, 0, 6], [7, 5, 8]]  # Initial state of the puzzle
goal_state = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]  # Goal state

solver = IterativeDeepeningSolver(start_state, goal_state)
solution_path = solver.iterative_deepening_search()

if solution_path:
    print("Solution found!")
    for step in solution_path:
        for row in step:
            print(row)
        print("----")
else:
    print("No solution found.")
