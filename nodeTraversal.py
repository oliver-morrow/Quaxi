from globals import PATH_DATA
from pathfinding import a_star_search, find_nearest_node
import globals

class NodeTraversal:
    def __init__(self):
        # This will be populated from main.py
        self.short_term = None
        
    def set_controller(self, controller):
        self.short_term = controller

    def transitionState(self, state):
        if self.short_term:
            self.short_term.stateTransition = False
            self.short_term.CURRENT_STATE = state
            self.short_term.stateStartIteration = 0
            self.short_term.relativeIteration = 0
        else:
            # Fallback to global vars for backward compatibility
            import ShortTermController as ShortTerm
            ShortTerm.stateTransition = False
            ShortTerm.CURRENT_STATE = state
            ShortTerm.stateStartIteration = 0
            ShortTerm.relativeIteration = 0

    def executeInstruction(self, instruction):
        command, value = instruction
        
        if self.short_term:
            state = globals.State.Wait
            if command == "forwards" or command == "straight":
                state = globals.State.Forward
            elif command == "turn_right":
                state = globals.State.TurnR
            elif command == "turn_left":
                state = globals.State.TurnL
            elif command == "wait":
                state = globals.State.Wait
            
            self.transitionState(state)
            self.short_term.value = value

            while not self.short_term.stateTransition:
                # TODO update current position
                self.short_term.iteration()
                continue
        else:
            # Fallback to global vars for backward compatibility
            import ShortTermController as ShortTerm
            state = ShortTerm.State.Wait
            if command == "forwards" or command == "straight":
                state = ShortTerm.State.Forward
            elif command == "turn_right":
                state = ShortTerm.State.TurnR
            elif command == "turn_left":
                state = ShortTerm.State.TurnL
            elif command == "wait":
                state = ShortTerm.State.Wait
            
            self.transitionState(state)
            ShortTerm.value = value

            while not ShortTerm.stateTransition:
                # TODO update current position
                ShortTerm.iteration()
                continue

    def execute_path(self, start_node, dest_x, dest_y):
        """
        Finds the nearest node to (dest_x, dest_y), computes the path using A* search,
        and executes the movement instructions step by step.
        """
        # Find nearest destination node
        destination_node = find_nearest_node(dest_x, dest_y)
        print(f"🎯 Nearest node to ({dest_x}, {dest_y}) is: {destination_node}")

        # Get the path from start_node to destination_node
        path = a_star_search(start_node, destination_node)
        if not path or len(path) < 3:
            print("❌ No valid path found or path too short!")
            return
        
        print(f"\n🗺️  Generated Path: {' -> '.join(path)}\n")

        # Start traversal using the first node as the current node
        current_node = path[0]
        intersection_node = path[1]
        next_node = path[2]

        index = 0
        while index < len(path) - 2:
            if intersection_node not in PATH_DATA or "paths" not in PATH_DATA[intersection_node]:
                print(f"⚠️ No path data for {intersection_node}, skipping...")
            else:
                key = (current_node, next_node)
                if key in PATH_DATA[intersection_node]["paths"]:
                    movements = PATH_DATA[intersection_node]["paths"][key]
                    print(f"🚦 Moving from '{current_node}' to '{next_node}' via '{intersection_node}'")
                    for step in movements:
                        if isinstance(step, list) and len(step) == 2:
                            command, value = step
                            self.executeInstruction(step)
                            print(f"➡️  {command.capitalize()} {value} units/degrees")
                        elif isinstance(step, str):
                            print(f"🛑 Special Instruction: {step}")
                    print("✅ Instruction sequence completed.\n")
                else:
                    print(f"⚠️ No movement instructions for {current_node} -> {next_node} via {intersection_node}, proceeding without instructions.")
            
            # Shift indices for next iteration
            index += 1
            if index + 2 < len(path):
                current_node = path[index]
                intersection_node = path[index + 1]
                next_node = path[index + 2]
            else:
                break
                
        if self.short_term:
            self.transitionState(globals.State.Wait)
            self.short_term.iteration()
        else:
            import ShortTermController as ShortTerm
            self.transitionState(ShortTerm.State.Wait)
            ShortTerm.iteration()

# ✅ **Test the function**
if __name__ == "__main__":
    traversal = NodeTraversal()
    ShortTerm.init()

    # Example: Start at 'PondsideAve.:QuackSt' and go to (585, 135)
    traversal.execute_path("PondsideAve.:QuackSt", 585, 135)
