import math
from globals import NODE_COORDS,MAP_GRAPH,NODES_DATA

# ----- 1) All Node & Road Data in a Single Array -----

def heuristic(node_a, node_b):
    (x1, y1) = NODE_COORDS[node_a]
    (x2, y2) = NODE_COORDS[node_b]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def a_star_search(start_node, goal_node):
    open_set = {start_node}
    came_from = {}
    
    g_score = {node: float('inf') for node in MAP_GRAPH}
    g_score[start_node] = 0.0
    
    f_score = {node: float('inf') for node in MAP_GRAPH}
    f_score[start_node] = heuristic(start_node, goal_node)
    
    while open_set:
        current = min(open_set, key=lambda n: f_score[n])
        if current == goal_node:
            return reconstruct_path(came_from, current)
        
        open_set.remove(current)
        for (neighbor, edge_cost) in MAP_GRAPH[current]:
            tentative_g = g_score[current] + edge_cost
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal_node)
                open_set.add(neighbor)
    
    return []

def find_nearest_node(x, y):
    min_dist = float('inf')
    nearest = None
    for node, (nx, ny) in NODE_COORDS.items():
        dist = math.hypot(nx - x, ny - y)
        if dist < min_dist:
            min_dist = dist
            nearest = node
    return nearest

if __name__ == "__main__":
    start = "PondsideAve.:QuackSt"
    goal_x, goal_y = 585, 135

    goal_node = find_nearest_node(goal_x, goal_y)
    print(f"Nearest node to ({goal_x}, {goal_y}) is: {goal_node}")

    path = a_star_search(start, goal_node)
    if path:
        print(f"Full path from '{start}' to '{goal_node}':")
        print(" -> ".join(path))
    else:
        print("No path found.")
