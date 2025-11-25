import heapq
import math

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

class Planner:
    def __init__(self, resolution=1.0, robot_radius=2.0):
        self.resolution = resolution
        self.robot_radius = robot_radius

    def plan(self, start_x, start_y, goal_x, goal_y, env):
        """
        A* algorithm
        """
        # Convert to grid coordinates
        sx = int(start_x / self.resolution)
        sy = int(start_y / self.resolution)
        gx = int(goal_x / self.resolution)
        gy = int(goal_y / self.resolution)

        start_node = Node(sx, sy, 0.0, -1)
        goal_node = Node(gx, gy, 0.0, -1)

        open_set = dict()
        closed_set = dict()
        open_set[(sx, sy)] = start_node

        while True:
            if not open_set:
                print("Open set empty, no path found.")
                return None

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(open_set[o], goal_node))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            # Expand neighbors
            for move_x, move_y, move_cost in self.get_motion_model():
                node_x = current.x + move_x
                node_y = current.y + move_y
                node_cost = current.cost + move_cost

                if not self.verify_node(node_x, node_y, env):
                    continue

                if (node_x, node_y) in closed_set:
                    continue

                if (node_x, node_y) not in open_set:
                    new_node = Node(node_x, node_y, node_cost, c_id)
                    open_set[(node_x, node_y)] = new_node
                else:
                    if open_set[(node_x, node_y)].cost > node_cost:
                        open_set[(node_x, node_y)].cost = node_cost
                        open_set[(node_x, node_y)].parent_index = c_id

        return self.calc_final_path(goal_node, closed_set)

    def calc_heuristic(self, n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def verify_node(self, x, y, env):
        # Check boundaries
        if x < 0 or y < 0 or x >= env.cols or y >= env.rows:
            return False
        
        # Check collision
        if env.grid[y][x] == 1:
            return False
            
        # Check robot radius (simple check: if any neighbor is obstacle)
        # For better collision checking, we should check a circle around the node
        # Here we just check the grid cell itself for simplicity as per requirements
        return True

    def get_motion_model(self):
        # dx, dy, cost
        return [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)]
        ]

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [goal_node.x * self.resolution], [goal_node.y * self.resolution]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x * self.resolution)
            ry.append(n.y * self.resolution)
            parent_index = n.parent_index
        return list(zip(rx, ry))[::-1] # Return reversed path (start -> goal)
