import sys
import os
import matplotlib.pyplot as plt
import numpy as np
import math

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from src.simulation.environment import Environment
from src.robot.rover import Rover
from src.navigation.planner import Planner
from src.navigation.controller import Controller
from src.perception.slam import SLAM

def main():
    print("Starting Warehouse Inventory Rover Simulation (Level 1 - 2D)...")
    
    # Initialize environment
    env = Environment(width=100, height=100, resolution=1.0)
    # Add some shelves (obstacles)
    env.add_obstacle(20, 20, 10, 60)
    env.add_obstacle(50, 20, 10, 60)
    env.add_obstacle(80, 20, 10, 60)
    
    # Initialize robot
    rover = Rover(x=10, y=10, theta=0.0)
    
    # Initialize modules
    planner = Planner(resolution=1.0)
    controller = Controller()
    slam = SLAM(width=100, height=100, resolution=1.0)
    
    # Plan a path
    goal_x, goal_y = 90, 90
    print(f"Planning path from ({rover.x}, {rover.y}) to ({goal_x}, {goal_y})...")
    path = planner.plan(rover.x, rover.y, goal_x, goal_y, env)
    
    if not path:
        print("No path found!")
        return

    print(f"Path found with {len(path)} waypoints.")
    
    # Setup visualization
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    
    from matplotlib.animation import FuncAnimation
    
    path_x, path_y = zip(*path) if path else ([], [])
    
    def update(frame):
        # 1. Perception
        rover.update(dt=0.1, env=env)
        slam.update(rover.x, rover.y, rover.theta, rover.lidar_scan)
        
        # 2. Control
        v, w = controller.compute_control(rover, path)
        rover.v = v
        rover.w = w
        
        # 3. Check goal
        dist_to_goal = math.hypot(goal_x - rover.x, goal_y - rover.y)
        if dist_to_goal < 2.0:
            print("Goal reached!")
            # Stop robot
            rover.v = 0
            rover.w = 0
            
        # 4. Visualization
        # Ground truth
        env.render(ax1, robot=rover)
        if path:
            ax1.plot(path_x, path_y, 'r--', label='Planned Path')
        ax1.set_title("Ground Truth")
        ax1.legend(loc='upper right')
        
        # SLAM Map
        ax2.clear()
        ax2.imshow(slam.map, cmap='Greys', origin='lower', extent=[0, 100, 0, 100], vmin=0, vmax=1)
        ax2.set_title("SLAM Map (Occupancy Grid)")
        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        
        return ax1, ax2

    print("Running simulation...")
    anim = FuncAnimation(fig, update, frames=200, interval=50, blit=False)
    plt.show()

if __name__ == "__main__":
    main()
