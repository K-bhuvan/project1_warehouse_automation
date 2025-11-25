import numpy as np
import math

class SLAM:
    def __init__(self, width, height, resolution=1.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.cols = int(width / resolution)
        self.rows = int(height / resolution)
        # 0.5 = unknown, 1.0 = occupied, 0.0 = free
        self.map = np.full((self.rows, self.cols), 0.5)

    def update(self, robot_x, robot_y, robot_theta, scan_data):
        # Simple mapping: update cells along the ray
        # Log-odds would be better, but simple increment/decrement is fine for now
        
        for angle, dist in scan_data:
            # Ray end point
            end_x = robot_x + dist * math.cos(angle)
            end_y = robot_y + dist * math.sin(angle)
            
            # Trace ray
            self.trace_ray(robot_x, robot_y, end_x, end_y, dist < 20.0) # 20.0 is max range

    def trace_ray(self, x0, y0, x1, y1, hit):
        # Bresenham's line algorithm or similar
        # Here we use a simple sampling approach
        steps = int(math.hypot(x1-x0, y1-y0) / (self.resolution / 2))
        if steps == 0: return
        
        for i in range(steps):
            t = i / steps
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            
            c = int(x / self.resolution)
            r = int(y / self.resolution)
            
            if 0 <= c < self.cols and 0 <= r < self.rows:
                # Mark as free
                self.map[r, c] = max(0.0, self.map[r, c] - 0.1)
        
        if hit:
            # Mark end as occupied
            c = int(x1 / self.resolution)
            r = int(y1 / self.resolution)
            if 0 <= c < self.cols and 0 <= r < self.rows:
                self.map[r, c] = min(1.0, self.map[r, c] + 0.3)
