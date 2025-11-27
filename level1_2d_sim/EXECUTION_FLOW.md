# Warehouse Rover Simulation - Execution Flow Documentation

## Overview
This document explains the complete execution flow of the 2D warehouse inventory rover simulation, from initial startup through runtime operation.

---

## 🚀 Execution Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                    START: python src/main.py                        │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    PHASE 1: INITIALIZATION                          │
├─────────────────────────────────────────────────────────────────────┤
│  1. Import Dependencies                                             │
│     • matplotlib.pyplot, numpy, math, sys, os                       │
│     • Custom modules: Environment, Rover, Planner, Controller, SLAM │
│                                                                     │
│  2. main() Function Starts                                          │
│     └─> Print startup message                                       │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  3. Create Environment (100x100 grid)                               │
│     Environment.__init__()                                          │
│     ├─> Set width=100, height=100, resolution=1.0                  │
│     ├─> Create grid: np.zeros((100, 100))                          │
│     └─> Initialize empty obstacles list                            │
│                                                                     │
│  4. Add Obstacles (Warehouse Shelves)                               │
│     env.add_obstacle() called 3 times                               │
│     ├─> Shelf 1: (x=20, y=20, w=10, h=60)                          │
│     ├─> Shelf 2: (x=50, y=20, w=10, h=60)                          │
│     └─> Shelf 3: (x=80, y=20, w=10, h=60)                          │
│         └─> Updates grid cells to mark obstacles (1=occupied)      │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  5. Create Rover (Robot)                                            │
│     Rover.__init__(x=10, y=10, theta=0.0)                          │
│     ├─> Set initial position (10, 10)                              │
│     ├─> Set initial heading theta=0.0 (facing right)               │
│     ├─> Initialize velocities: v=0.0, w=0.0                        │
│     └─> Create Lidar sensor                                        │
│         Lidar.__init__(max_range=20.0, num_rays=36)                │
│         └─> 360° FOV with 36 rays                                  │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  6. Initialize Navigation & Perception Modules                      │
│     ├─> Planner(resolution=1.0, robot_radius=2.0)                  │
│     │   └─> Prepares A* path planning                              │
│     ├─> Controller(kp_linear=1.0, kp_angular=2.0)                  │
│     │   └─> Sets up proportional control gains                     │
│     └─> SLAM(width=100, height=100, resolution=1.0)                │
│         └─> Creates occupancy map: np.full((100,100), 0.5)         │
│            (0.5 = unknown, 1.0 = occupied, 0.0 = free)             │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    PHASE 2: PATH PLANNING                           │
├─────────────────────────────────────────────────────────────────────┤
│  7. Set Goal Position: (80, 90)                                     │
│                                                                     │
│  8. Plan Path using A* Algorithm                                    │
│     planner.plan(start=(10,10), goal=(80,90), env)                 │
│     │                                                               │
│     ├─> Convert to grid coordinates                                │
│     │   start_grid = (10, 10), goal_grid = (80, 90)                │
│     │                                                               │
│     ├─> Initialize open_set & closed_set                           │
│     │                                                               │
│     ├─> A* Main Loop:                                              │
│     │   ┌─────────────────────────────────────────┐                │
│     │   │ While open_set not empty:              │                │
│     │   │  1. Find node with lowest f-score      │                │
│     │   │     f = g + h                          │                │
│     │   │     g = cost from start                │                │
│     │   │     h = heuristic(current, goal)       │                │
│     │   │                                        │                │
│     │   │  2. Check if goal reached              │                │
│     │   │     → YES: break                       │                │
│     │   │     → NO: continue                     │                │
│     │   │                                        │                │
│     │   │  3. Move current to closed_set         │                │
│     │   │                                        │                │
│     │   │  4. Expand neighbors (8 directions)    │                │
│     │   │     For each neighbor:                 │                │
│     │   │     a) verify_node():                  │                │
│     │   │        - Check boundaries              │                │
│     │   │        - Check collision               │                │
│     │   │        - Check safety radius           │                │
│     │   │     b) Update costs if valid           │                │
│     │   │     c) Add to open_set                 │                │
│     │   └─────────────────────────────────────────┘                │
│     │                                                               │
│     └─> calc_final_path():                                         │
│         Backtrack from goal to start using parent pointers         │
│         Returns: [(x1,y1), (x2,y2), ..., (goal_x, goal_y)]        │
│                                                                     │
│  9. Print path info                                                 │
│     "Path found with N waypoints"                                   │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    PHASE 3: VISUALIZATION SETUP                     │
├─────────────────────────────────────────────────────────────────────┤
│  10. Create Figure with 2 Subplots                                  │
│      fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))         │
│      ├─> ax1: Ground Truth View                                    │
│      └─> ax2: SLAM Map View                                        │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    PHASE 4: SIMULATION LOOP                         │
│                   (200 frames @ 50ms interval)                      │
├─────────────────────────────────────────────────────────────────────┤
│  11. update(frame) - Called by FuncAnimation                        │
│      ┌────────────────────────────────────────────────────┐         │
│      │  FOR EACH FRAME (0 to 199):                       │         │
│      │                                                    │         │
│      │  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  │         │
│      │  ┃  STEP 1: PERCEPTION                        ┃  │         │
│      │  ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  │         │
│      │  rover.update(dt=0.1, env)                        │         │
│      │  ├─> Update robot kinematics:                     │         │
│      │  │   x += v * cos(theta) * dt                     │         │
│      │  │   y += v * sin(theta) * dt                     │         │
│      │  │   theta += w * dt                              │         │
│      │  └─> Perform Lidar scan:                          │         │
│      │      lidar.scan(x, y, theta, env)                 │         │
│      │      ├─> Cast 36 rays in 360°                     │         │
│      │      └─> For each ray:                            │         │
│      │          cast_ray():                              │         │
│      │          ├─> Step along ray direction             │         │
│      │          ├─> Check for obstacles                  │         │
│      │          └─> Return distance to hit               │         │
│      │                                                    │         │
│      │  slam.update(x, y, theta, lidar_scan)             │         │
│      │  ├─> For each lidar ray:                          │         │
│      │  │   └─> trace_ray(start, end, hit)               │         │
│      │  │       ├─> Mark cells along ray as free         │         │
│      │  │       │   (decrement probability)              │         │
│      │  │       └─> Mark endpoint as occupied            │         │
│      │  │           (increment probability)              │         │
│      │  └─> Updates occupancy grid map                   │         │
│      │                                                    │         │
│      │  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  │         │
│      │  ┃  STEP 2: CONTROL                           ┃  │         │
│      │  ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  │         │
│      │  controller.compute_control(rover, path)          │         │
│      │  ├─> Find closest point on path                   │         │
│      │  ├─> Select lookahead target (5.0m ahead)         │         │
│      │  ├─> Calculate errors:                            │         │
│      │  │   heading_error = target_heading - robot.theta │         │
│      │  │   distance_error = dist(robot, target)         │         │
│      │  ├─> Compute control commands:                    │         │
│      │  │   v = kp_linear * distance_error               │         │
│      │  │   w = kp_angular * heading_error               │         │
│      │  ├─> Apply constraints:                           │         │
│      │  │   v: slow down if heading error large          │         │
│      │  │   v: capped [0, 5.0]                          │         │
│      │  │   w: capped [-2.0, 2.0]                       │         │
│      │  └─> Return (v, w)                                │         │
│      │                                                    │         │
│      │  rover.v = v  # Set linear velocity               │         │
│      │  rover.w = w  # Set angular velocity              │         │
│      │                                                    │         │
│      │  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  │         │
│      │  ┃  STEP 3: GOAL CHECK                        ┃  │         │
│      │  ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  │         │
│      │  dist_to_goal = hypot(goal - robot.position)      │         │
│      │  IF dist_to_goal < 2.0:                           │         │
│      │     print("Goal reached!")                        │         │
│      │     rover.v = 0  # Stop                           │         │
│      │     rover.w = 0                                   │         │
│      │                                                    │         │
│      │  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  │         │
│      │  ┃  STEP 4: VISUALIZATION                     ┃  │         │
│      │  ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  │         │
│      │  # Left Plot (Ground Truth):                      │         │
│      │  env.render(ax1, robot=rover)                     │         │
│      │  ├─> Draw obstacle grid                           │         │
│      │  ├─> Draw robot (blue circle)                     │         │
│      │  ├─> Draw heading arrow (red)                     │         │
│      │  └─> Draw lidar rays (green)                      │         │
│      │  Plot planned path (red dashed line)              │         │
│      │                                                    │         │
│      │  # Right Plot (SLAM Map):                         │         │
│      │  ax2.imshow(slam.map)                             │         │
│      │  └─> Display occupancy grid                       │         │
│      │      Gray = unknown, White = free, Black = occupied│        │
│      │                                                    │         │
│      └────────────────────────────────────────────────────┘         │
│                                                                     │
│  Animation runs for 200 frames × 50ms = 10 seconds                 │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    PHASE 5: SAVE & DISPLAY                          │
├─────────────────────────────────────────────────────────────────────┤
│  12. Create results directory                                       │
│      os.makedirs('results', exist_ok=True)                         │
│                                                                     │
│  13. Save animation as GIF                                          │
│      anim.save('results/simulation.gif', fps=20)                   │
│                                                                     │
│  14. Save final figure snapshot                                     │
│      plt.savefig('results/Figure_1.png', dpi=150)                  │
│                                                                     │
│  15. Display interactive plot                                       │
│      plt.show()                                                     │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
                         ┌───┴───┐
                         │  END  │
                         └───────┘
```

---

## 📋 Detailed Component Breakdown

### 1. **Environment Module** (`simulation/environment.py`)
**Purpose**: Manages the warehouse layout and obstacles

#### Key Data Structures:
- `grid`: 2D numpy array (100×100) storing obstacle information
  - `0` = free space
  - `1` = obstacle (shelf)
  
#### Key Methods:
- `__init__(width, height, resolution)`: Initialize grid
- `add_obstacle(x, y, w, h)`: Add rectangular obstacles (shelves)
- `is_obstacle(x, y)`: Check if coordinate contains obstacle
- `render(ax, robot)`: Visualize environment with robot

---

### 2. **Rover Module** (`robot/rover.py`)
**Purpose**: Represents the warehouse robot and its onboard sensor

#### Components:
##### A. Rover Class
- **State Variables**:
  - `x, y`: Position in meters
  - `theta`: Heading angle in radians
  - `v`: Linear velocity (m/s)
  - `w`: Angular velocity (rad/s)
  
- **update(dt, env)**: Updates robot state using kinematic model
  ```python
  x_new = x + v * cos(theta) * dt
  y_new = y + v * sin(theta) * dt
  theta_new = theta + w * dt
  ```

##### B. Lidar Class
- **Specifications**:
  - Max range: 20 meters
  - Number of rays: 36 (10° spacing)
  - FOV: 360°
  
- **scan(x, y, theta, env)**: Performs full 360° scan
- **cast_ray(x, y, angle, env)**: Ray-casting for single beam
  - Steps along ray direction
  - Returns distance to first obstacle hit

---

### 3. **Planner Module** (`navigation/planner.py`)
**Purpose**: Computes collision-free paths using A* algorithm

#### A* Implementation Details:

##### Data Structures:
- **Node**: Stores (x, y, cost, parent_index)
- **open_set**: Nodes to be explored (frontier)
- **closed_set**: Already explored nodes

##### Algorithm Flow:
1. **Initialization**
   - Convert world coordinates to grid coordinates
   - Create start and goal nodes
   
2. **Main Loop**
   ```
   while open_set not empty:
     1. Select node with lowest f-score
        f = g + h
        g = actual cost from start
        h = heuristic (Euclidean distance to goal)
     
     2. If goal reached → reconstruct path
     
     3. Move current to closed_set
     
     4. Expand neighbors (8 directions):
        - Up, Down, Left, Right (cost = 1)
        - Diagonals (cost = √2)
     
     5. For each neighbor:
        - verify_node(): Check if valid
        - Update costs if better path found
        - Add to open_set
   ```

3. **Node Verification** (`verify_node`)
   - **Boundary check**: Is node within grid?
   - **Collision check**: Is node on obstacle?
   - **Safety margin**: Check circular buffer around robot
     - Buffer radius = `robot_radius` (2.0 meters)
     - Ensures safe clearance from obstacles

4. **Path Reconstruction**
   - Backtrack from goal using parent pointers
   - Returns list of waypoints: `[(x1,y1), (x2,y2), ..., (goal_x, goal_y)]`

---

### 4. **Controller Module** (`navigation/controller.py`)
**Purpose**: Generates velocity commands to follow planned path

#### Pure Pursuit Control Algorithm:

##### Step 1: Find Position on Path
- Locate closest waypoint to current robot position
- Distance metric: Euclidean distance

##### Step 2: Lookahead Target Selection
- Lookahead distance: 5.0 meters
- Find first waypoint beyond lookahead distance
- If none found, use final goal

##### Step 3: Error Calculation
```python
dx = target_x - robot.x
dy = target_y - robot.y
target_heading = atan2(dy, dx)

heading_error = target_heading - robot.theta
distance_error = sqrt(dx² + dy²)
```

##### Step 4: Proportional Control
```python
v = kp_linear * distance_error     # Linear velocity
w = kp_angular * heading_error     # Angular velocity
```

##### Step 5: Safety Constraints
- **Heading error correction**: If `|heading_error| > 0.5 rad`, slow down (`v *= 0.1`)
- **Velocity limits**:
  - Linear: `v ∈ [0, 5.0]` m/s
  - Angular: `w ∈ [-2.0, 2.0]` rad/s
- **Goal proximity**: If within 1m of goal, stop completely

---

### 5. **SLAM Module** (`perception/slam.py`)
**Purpose**: Builds occupancy grid map from Lidar data

#### Occupancy Grid Mapping:

##### Map Representation:
- Grid size: 100×100 cells
- Cell values: `[0.0, 1.0]`
  - `0.0` = Free space
  - `0.5` = Unknown
  - `1.0` = Occupied

##### Update Algorithm:
For each Lidar ray:
1. **Trace Ray** (`trace_ray`)
   - Sample points along ray from robot to endpoint
   - Step size: `resolution / 2`
   
2. **Mark Free Space**
   - All cells along ray (except endpoint): probability -= 0.1
   - Clamped to minimum 0.0
   
3. **Mark Occupied Space**
   - If ray hit obstacle (dist < max_range):
     - Endpoint cell: probability += 0.3
     - Clamped to maximum 1.0

##### Rationale:
- **Free space**: "I can see through these cells"
- **Occupied**: "Ray stopped here, obstacle detected"
- **Probabilistic**: Multiple scans accumulate evidence

---

## 🔄 Runtime Data Flow

### Single Iteration Timeline (0.1 seconds):

```
Time: t
│
├─ [0ms] Robot at position (x, y, θ)
│        Velocities: (v, w)
│
├─ [10ms] Lidar scan
│         └─> Returns 36 distance measurements
│
├─ [20ms] SLAM update
│         └─> Occupancy grid updated
│
├─ [30ms] Controller computes (v, w)
│         ├─> Find lookahead target
│         ├─> Calculate errors
│         └─> Apply control law
│
├─ [40ms] Apply velocities to robot
│         rover.v = v
│         rover.w = w
│
├─ [50ms] Robot motion update
│         ├─> x += v * cos(θ) * dt
│         ├─> y += v * sin(θ) * dt
│         └─> θ += w * dt
│
├─ [60ms] Goal check
│         └─> If close to goal: v=0, w=0
│
├─ [70ms] Visualization update
│         ├─> Render ground truth (ax1)
│         └─> Render SLAM map (ax2)
│
└─ [100ms] Frame complete
          Go to Time: t+1
```

---

## 📊 Key Parameters Summary

| Component | Parameter | Value | Purpose |
|-----------|-----------|-------|---------|
| **Environment** | Width × Height | 100 × 100 m | Warehouse size |
| | Resolution | 1.0 m | Grid cell size |
| **Robot** | Initial Position | (10, 10) m | Start location |
| | Initial Heading | 0° (East) | Starting direction |
| | Max Linear Vel | 5.0 m/s | Speed limit |
| | Max Angular Vel | ±2.0 rad/s | Turn rate limit |
| **Lidar** | Max Range | 20 m | Sensor range |
| | Number of Rays | 36 | Angular resolution (10°) |
| | FOV | 360° | Full coverage |
| **Planner** | Robot Radius | 2.0 m | Safety margin |
| | Motion Model | 8-connected | Allows diagonal movement |
| **Controller** | Lookahead Distance | 5.0 m | Pure pursuit parameter |
| | kp_linear | 1.0 | Linear velocity gain |
| | kp_angular | 2.0 | Angular velocity gain |
| **SLAM** | Initial Probability | 0.5 | Unknown state |
| | Free Update | -0.1 | Decrease occupancy |
| | Occupied Update | +0.3 | Increase occupancy |
| **Simulation** | dt | 0.1 s | Timestep |
| | Frames | 200 | Total duration: 20s |
| | Animation Interval | 50 ms | Visualization refresh |

---

## 🎯 Goal Detection Logic

```python
distance_to_goal = sqrt((goal_x - robot.x)² + (goal_y - robot.y)²)

if distance_to_goal < 2.0:  # Within 2 meters
    print("Goal reached!")
    robot.v = 0.0  # Stop
    robot.w = 0.0
```

---

## 📁 Output Files

After simulation completes:
1. **`results/simulation.gif`**: Animated visualization (20 fps)
2. **`results/Figure_1.png`**: Final snapshot (150 dpi)

Both show:
- **Left panel**: Ground truth with obstacles, robot, lidar rays, planned path
- **Right panel**: SLAM-generated occupancy map

---

## 🔍 Important Implementation Notes

### 1. **Coordinate Systems**
- **World coordinates**: Continuous (meters)
- **Grid coordinates**: Discrete (cells)
- Conversion: `grid_coord = int(world_coord / resolution)`

### 2. **Angle Normalization**
```python
while theta > π:  theta -= 2π
while theta < -π: theta += 2π
```
Keeps angle in range `[-π, π]`

### 3. **Path Following Strategy**
- **Close to path**: Look ahead 5m for smooth motion
- **Far from path**: Find closest waypoint first
- **Near goal**: Reduce velocity to ensure precision

### 4. **Safety Mechanisms**
- **Planning**: Include robot radius in collision checking
- **Control**: Slow down for large heading errors
- **Motion**: Cap all velocities to safe limits

---

## 🐛 Debugging Tips

### To trace execution:
1. **Add prints in update() loop**:
   ```python
   print(f"Frame {frame}: Robot at ({rover.x:.2f}, {rover.y:.2f}), v={v:.2f}, w={w:.2f}")
   ```

2. **Check path validity**:
   ```python
   if not path:
       print("Path planning failed!")
       return
   ```

3. **Monitor goal progress**:
   ```python
   dist = math.hypot(goal_x - rover.x, goal_y - rover.y)
   print(f"Distance to goal: {dist:.2f}m")
   ```

---

## 🚦 Execution States

The robot transitions through these states:

1. **PLANNING**: Computing optimal path (once at start)
2. **NAVIGATING**: Following path towards goal
3. **CORRECTING**: Adjusting heading to align with path
4. **APPROACHING**: Close to goal, reducing speed
5. **STOPPED**: Goal reached, velocities = 0

---

This flow document provides a complete picture of how your warehouse rover simulation works from start to finish! Each component plays a specific role in the autonomous navigation pipeline.
