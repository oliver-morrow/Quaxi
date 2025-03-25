import math
import time
from globals import NODE_COORDS, MAP_GRAPH, PATH_DATA, State

class PathFinder:
    def __init__(self, car_controller=None):
        """Initialize PathFinder with optional car controller"""
        self.car_controller = car_controller
        self.current_node = None
        self.is_navigating = False
        
    def set_car_controller(self, controller):
        """Set the car controller used for executing movements"""
        self.car_controller = controller
        
    def heuristic(self, node_a, node_b):
        """Calculate euclidean distance between two nodes"""
        (x1, y1) = NODE_COORDS[node_a]
        (x2, y2) = NODE_COORDS[node_b]
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def a_star_search(self, start_node, goal_node):
        """A* search algorithm for finding optimal path between nodes"""
        # Initialize open set with start node
        open_set = {start_node}
        came_from = {}
        
        # Initialize distance scores
        g_score = {node: float('inf') for node in MAP_GRAPH}
        g_score[start_node] = 0.0
        
        # Initialize total scores (distance + heuristic)
        f_score = {node: float('inf') for node in MAP_GRAPH}
        f_score[start_node] = self.heuristic(start_node, goal_node)
        
        # Main A* loop
        while open_set:
            # Get node with lowest f_score
            current = min(open_set, key=lambda n: f_score[n])
            
            # Check if we've reached the goal
            if current == goal_node:
                return self.reconstruct_path(came_from, current)
            
            # Process current node
            open_set.remove(current)
            for (neighbor, edge_cost) in MAP_GRAPH[current]:
                # Calculate tentative distance to neighbor
                tentative_g = g_score[current] + edge_cost
                # If we found a better path to neighbor
                if tentative_g < g_score[neighbor]:
                    # Record the path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_node)
                    open_set.add(neighbor)
        
        # No path found
        return []

    def find_nearest_node(self, x, y):
        """Find the closest node to given coordinates"""
        min_dist = float('inf')
        nearest = None
        for node, (nx, ny) in NODE_COORDS.items():
            dist = math.hypot(nx - x, ny - y)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest
    
    def execute_movement(self, command, value):
        """Execute a single movement command"""
        if not self.car_controller:
            print(f"No car controller - would execute: {command} {value}")
            return True
            
        print(f"Executing: {command} {value}")
        
        if command == "forwards" or command == "straight":
            # Convert distance to time (assuming constant speed)
            duration = value / 20.0  # This factor may need tuning
            self.car_controller.turn_right(angle=0, speed=30)
            time.sleep(duration)
            self.car_controller.stop()
            
        elif command == "turn_right":
            # Turn right by given angle
            self.car_controller.turn_right(angle=min(value, 35), speed=20)
            time.sleep(1.0)  # Short turn duration
            self.car_controller.stop()
            
        elif command == "turn_left":
            # Turn left by given angle
            self.car_controller.turn_right(angle=-min(value, 35), speed=20)
            time.sleep(1.0)  # Short turn duration
            self.car_controller.stop()
            
        elif command == "wait":
            # Wait for given duration
            time.sleep(value)
            
        # Add small delay between commands
        time.sleep(0.5)
        return True
        
    def navigate_to(self, start_node, goal_x, goal_y):
        """Navigate from start node to coordinates using A*"""
        # Find nearest node to goal coordinates
        goal_node = self.find_nearest_node(goal_x, goal_y)
        print(f"Navigating from '{start_node}' to '{goal_node}' at ({goal_x}, {goal_y})")
        
        # Get path using A*
        path = self.a_star_search(start_node, goal_node)
        if not path or len(path) < 3:
            print("No valid path found or path too short!")
            return False
            
        print(f"Path found: {' -> '.join(path)}")
        self.is_navigating = True
        
        # Execute path node by node
        for i in range(len(path) - 2):
            current_node = path[i]
            intersection_node = path[i + 1]
            next_node = path[i + 2]
            
            print(f"Moving from '{current_node}' to '{next_node}' via '{intersection_node}'")
            
            # Skip if no path data for intersection
            if intersection_node not in PATH_DATA or "paths" not in PATH_DATA[intersection_node]:
                print(f"No path data for {intersection_node}, skipping...")
                continue
                
            # Get movement instructions for this segment
            key = (current_node, next_node)
            if key in PATH_DATA[intersection_node]["paths"]:
                movements = PATH_DATA[intersection_node]["paths"][key]
                
                # Execute each movement instruction
                for step in movements:
                    if isinstance(step, list) and len(step) == 2:
                        # Regular movement command [command, value]
                        self.execute_movement(step[0], step[1])
                    elif isinstance(step, str):
                        # Special instruction like "STOP SIGN"
                        print(f"Special instruction: {step}")
                        if step == "STOP SIGN":
                            # Stop at sign
                            self.car_controller.stop()
                            time.sleep(3)  # Wait at stop sign
            else:
                print(f"No movement instructions for {current_node} -> {next_node}, skipping...")
        
        self.is_navigating = False
        self.current_node = goal_node
        return True
            
    def stop_navigation(self):
        """Stop current navigation"""
        self.is_navigating = False
        if self.car_controller:
            self.car_controller.stop()


# Add standalone functions for backward compatibility
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

# Example usage
if __name__ == "__main__":
    pathfinder = PathFinder()
    
    # Find path from start to goal
    start_node = "PondsideAve.:QuackSt"
    goal_x, goal_y = 585, 135
    
    # Find the nearest node to goal coordinates
    goal_node = pathfinder.find_nearest_node(goal_x, goal_y)
    print(f"Nearest node to ({goal_x}, {goal_y}) is: {goal_node}")
    
    # Get the path
    path = pathfinder.a_star_search(start_node, goal_node)
    if path:
        print(f"Path from '{start_node}' to '{goal_node}':")
        print(" -> ".join(path))
    else:
        print("No path found.")
