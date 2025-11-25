import math

class Controller:
    def __init__(self, kp_linear=1.0, kp_angular=2.0):
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular

    def compute_control(self, robot, path):
        if not path or len(path) < 2:
            return 0.0, 0.0

        # Find closest point on path
        closest_idx = 0
        min_dist = float('inf')
        for i, (px, py) in enumerate(path):
            dist = math.hypot(px - robot.x, py - robot.y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Look ahead
        lookahead_dist = 5.0
        target_x, target_y = path[-1]
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            dist = math.hypot(px - robot.x, py - robot.y)
            if dist > lookahead_dist:
                target_x, target_y = px, py
                break
        
        # Calculate error
        dx = target_x - robot.x
        dy = target_y - robot.y
        target_heading = math.atan2(dy, dx)
        
        heading_error = target_heading - robot.theta
        # Normalize angle
        while heading_error > math.pi: heading_error -= 2*math.pi
        while heading_error < -math.pi: heading_error += 2*math.pi
        
        # print(f"Target: ({target_x:.2f}, {target_y:.2f}), Robot: ({robot.x:.2f}, {robot.y:.2f}, {robot.theta:.2f}), Error: {heading_error:.2f}")
        
        dist_error = math.hypot(dx, dy)
        
        # Control laws
        v = self.kp_linear * dist_error
        w = self.kp_angular * heading_error
        
        # Slow down if heading error is large
        if abs(heading_error) > 0.5:
            v *= 0.1
        
        # Cap velocities
        v = max(min(v, 5.0), 0.0) # Only move forward
        w = max(min(w, 2.0), -2.0)
        
        # If very close to goal, stop
        if math.hypot(path[-1][0] - robot.x, path[-1][1] - robot.y) < 1.0:
            v = 0.0
            w = 0.0
            
        return v, w
