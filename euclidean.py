def a_star(start, goal, obstacles):
    open_list = [start]
    closed_list = set()

    # g costs and f costs dictionaries
    g_cost = {start: 0}
    f_cost = {start: heuristic(start, goal)}

    while open_list:
        # Get node with the lowest f(n)
        current_node = min(open_list, key=lambda n: f_cost[n])

        # If goal is reached, return the path
        if current_node == goal:
            return reconstruct_path(came_from, current_node)

        open_list.remove(current_node)
        closed_list.add(current_node)

        # Evaluate neighbors
        for neighbor in get_neighbors(current_node, obstacles):
            if neighbor in closed_list:
                continue

            tentative_g_cost = g_cost[current_node] + distance(current_node, neighbor)

            if neighbor not in open_list:
                open_list.append(neighbor)
            elif tentative_g_cost >= g_cost[neighbor]:
                continue

            # This path is the best until now
            came_from[neighbor] = current_node
            g_cost[neighbor] = tentative_g_cost
            f_cost[neighbor] = g_cost[neighbor] + heuristic(neighbor, goal)

    return None  # If no path found

def get_neighbors(node, obstacles):
    neighbors = []  # Potential 8-directional neighbors (or 4 if restricted)

    for neighbor in all_possible_neighbors(node):
        if not is_in_obstacle(neighbor, obstacles):  # Skip if inside an obstacle
            neighbors.append(neighbor)

    return neighbors

def is_in_obstacle(point, obstacles):
    for obstacle in obstacles:
        if obstacle['type'] == 'rectangle':
            if is_point_in_rectangle(point.x, point.y, *obstacle['vertices']):
                return True
        elif obstacle['type'] == 'triangle':
            if is_point_in_triangle(point.x, point.y, *obstacle['vertices']):
                return True
        # Add more shapes if needed
    return False
