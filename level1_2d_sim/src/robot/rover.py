import math
import numpy as np

class Lidar:
    def __init__(self, max_range=20.0, num_rays=36):
        self.max_range = max_range
        self.num_rays = num_rays
        self.fov = 2 * math.pi # 360 degrees

    def scan(self, x, y, theta, env):
        scan_data = []
        angles = np.linspace(theta - self.fov/2, theta + self.fov/2, self.num_rays)
        
        for angle in angles:
            dist = self.cast_ray(x, y, angle, env)
            scan_data.append((angle, dist))
        return scan_data

    def cast_ray(self, x, y, angle, env):
        step_size = env.resolution / 2.0
        dist = 0
        curr_x, curr_y = x, y
        
        while dist < self.max_range:
            dist += step_size
            curr_x += step_size * math.cos(angle)
            curr_y += step_size * math.sin(angle)
            
            if env.is_obstacle(curr_x, curr_y):
                return dist
        
        return self.max_range

class Rover:
    def __init__(self, x, y, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.w = 0.0
        self.lidar = Lidar()
        self.lidar_scan = []

    def update(self, dt, env=None):
        # Simple kinematic model
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        
        # Normalize theta
        while self.theta > math.pi: self.theta -= 2*math.pi
        while self.theta < -math.pi: self.theta += 2*math.pi
        
        if env:
            self.lidar_scan = self.lidar.scan(self.x, self.y, self.theta, env)
