import numpy as np
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, width, height, resolution=1.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.cols = int(width / resolution)
        self.rows = int(height / resolution)
        self.grid = np.zeros((self.rows, self.cols), dtype=int) # 0: free, 1: obstacle
        self.obstacles = []

    def add_obstacle(self, x, y, w, h):
        self.obstacles.append({'x': x, 'y': y, 'w': w, 'h': h})
        # Update grid
        c_start = int(max(0, x / self.resolution))
        c_end = int(min(self.cols, (x + w) / self.resolution))
        r_start = int(max(0, y / self.resolution))
        r_end = int(min(self.rows, (y + h) / self.resolution))
        self.grid[r_start:r_end, c_start:c_end] = 1

    def is_obstacle(self, x, y):
        c = int(x / self.resolution)
        r = int(y / self.resolution)
        if 0 <= c < self.cols and 0 <= r < self.rows:
            return self.grid[r, c] == 1
        return True # Out of bounds is obstacle

    def render(self, ax, robot=None):
        ax.clear()
        # Draw obstacles
        ax.imshow(self.grid, cmap='Greys', origin='lower', extent=[0, self.width, 0, self.height])
        
        # Draw robot
        if robot:
            circle = plt.Circle((robot.x, robot.y), radius=2, color='blue')
            ax.add_patch(circle)
            # Draw heading
            arrow_len = 3.0
            ax.arrow(robot.x, robot.y, arrow_len*np.cos(robot.theta), arrow_len*np.sin(robot.theta), 
                     head_width=1, color='red')
            
            # Draw Lidar rays if available
            if hasattr(robot, 'lidar_scan') and robot.lidar_scan:
                for angle, dist in robot.lidar_scan:
                    if dist < robot.lidar.max_range:
                        lx = robot.x + dist * np.cos(angle)
                        ly = robot.y + dist * np.sin(angle)
                        ax.plot([robot.x, lx], [robot.y, ly], 'g-', alpha=0.2)

        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_aspect('equal')
