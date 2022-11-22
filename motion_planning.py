from math import sqrt


def neighbors(current):
    # define the list of 4 neighbors
    neighbor_coords = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    return [(current[0] + nbr[0], current[1] + nbr[1]) for nbr in neighbor_coords]


def heuristic_distance(candidate, goal):
    dx = abs(candidate[0] - goal[0])
    dy = abs(candidate[1] - goal[1])
    return (dx + dy) + min(dx, dy) * (sqrt(2) - 2)


def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]

    current = start  # Assign start to current (protects against empty set error)

    open_list = [(0, start)]
    closed_list = []
    path = []

    cost = {start: 0}  # Cost Dictionary
    parent = {start: "none"}  # Parent Dictionary

    # Check if goal is obstacle

    while open_list:
        open_list.sort()  # Sort list in ascending order
        current = open_list.pop(0)[1]
        closed_list.append(current)

        # Check if goal reached
        if current == goal:
            break

        for candidate in neighbors(current):
            if candidate in obstacles:  # Checked if candidate is not an obstacle
                continue
            if candidate in closed_list:  # Checked if candidate has been visited
                continue

            new_cost = cost[current] + 1
            if candidate not in cost or new_cost < cost[candidate]:
                cost[candidate] = new_cost
                parent[candidate] = current

                final_cost = heuristic_distance(candidate, goal) + cost[candidate]
                open_list.append((final_cost, candidate))

    while current != start:
        path.append(current)
        current = parent[current]

    path.reverse()  # Generate waypoints by reversing path list

    return path
