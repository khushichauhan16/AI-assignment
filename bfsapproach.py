# here's a bfs approach to the problem.

from collections import deque

class Node:
    def __init__(self, state, parent, move, depth):
        self.state = state
        self.parent = parent
        self.move = move
        self.depth = depth

def bfs(start_state, goal_state):
    queue = deque([Node(start_state, None, None, 0)])
    visited = set()
    moves = ['UP', 'DOWN', 'LEFT', 'RIGHT']

    while queue:
        current_node = queue.popleft()
        current_state = current_node.state

        if current_state == goal_state:
            return construct_path(current_node)

        visited.add(tuple(current_state))

        for move in moves:
            new_state = perform_move(current_state, move)
            if new_state and tuple(new_state) not in visited:
                new_node = Node(new_state, current_node, move, current_node.depth + 1)
                queue.append(new_node)
                visited.add(tuple(new_state))

    return None

def perform_move(state, move):
    new_state = state[:]
    index = new_state.index(0)

    if move == 'UP':
        if index not in [0, 1, 2]:  # Can't move up from the first row
            new_state[index], new_state[index - 3] = new_state[index - 3], new_state[index]
            return new_state
    elif move == 'DOWN':
        if index not in [6, 7, 8]:  # Can't move down from the last row
            new_state[index], new_state[index + 3] = new_state[index + 3], new_state[index]
            return new_state
    elif move == 'LEFT':
        if index not in [0, 3, 6]:  # Can't move left from the first column
            new_state[index], new_state[index - 1] = new_state[index - 1], new_state[index]
            return new_state
    elif move == 'RIGHT':
        if index not in [2, 5, 8]:  # Can't move right from the last column
            new_state[index], new_state[index + 1] = new_state[index + 1], new_state[index]
            return new_state

    return None

def construct_path(node):
    path = []
    states = []
    while node.parent is not None:
        path.append(node.move)
        states.append(node.state)
        node = node.parent
    path.reverse()
    states.reverse()
    return path, states

def print_state(state):
    print("\n".join([
        f"{state[0]} {state[1]} {state[2]}",
        f"{state[3]} {state[4]} {state[5]}",
        f"{state[6]} {state[7]} {state[8]}"
    ]))
    print()

def get_user_input():
    print("Enter the 3x3 grid for the initial state (use 0 for the empty space, space-separated):")
    start_state = [int(x) for x in input().split()]
    print("Enter the 3x3 grid for the goal state (use 0 for the empty space, space-separated):")
    goal_state = [int(x) for x in input().split()]
    return start_state, goal_state

# Main function to run the BFS-based 8-puzzle solver
start_state, goal_state = get_user_input()

if len(start_state) == 9 and len(goal_state) == 9:
    solution, states = bfs(start_state, goal_state)
    if solution:
        print("Solution found!")
        print("Initial state:")
        print_state(start_state)
        for move, state in zip(solution, states):
            print(f"Move: {move}")
            print_state(state)
        print("Goal state reached:")
        print_state(goal_state)
    else:
        print("No solution found.")
else:
    print("Invalid input. Please enter exactly 9 numbers for each state.")
