# Project 1: Warehouse Inventory Rover

A simulated autonomous mobile robot for warehouse inventory management, featuring 2D and 3D simulation environments with LiDAR sensing, A* path planning, and SLAM mapping capabilities.

## Project Overview

This project implements a differential-drive rover capable of autonomous navigation in a warehouse environment. The robot builds a map using SLAM, plans optimal paths using A* pathfinding, and follows those paths using a Pure Pursuit controller.

## Project Structure

```
project1_warehouse_automation/
├── level1_2d_sim/          # 2D Matplotlib-based simulation
│   └── src/
│       ├── main.py
│       ├── simulation/     # Environment, GridMap
│       ├── robot/          # Rover, Lidar
│       ├── navigation/     # A* Planner, Controller
│       └── perception/     # SLAM
├── level2_3d_sim/          # 3D PyBullet-based simulation
│   └── src/
│       ├── main.py
│       ├── simulation/     # PyBullet Environment
│       ├── robot/          # PyBullet Sensors
│       ├── navigation/     # A* Planner, Controller
│       └── perception/     # SLAM
│   └── urdf/               # Robot URDF model
└── requirements.txt        # Python dependencies
```

## Features

### Level 1: 2D Simulation
- **Visualization**: Matplotlib-based 2D rendering with real-time updates
- **Physics**: Simple kinematic model for differential drive
- **Sensors**: 2D LiDAR ray casting (360° scan, 36 rays)
- **Mapping**: Occupancy grid-based SLAM
- **Navigation**: A* path planning with Pure Pursuit controller

### Level 2: 3D Simulation
- **Visualization**: PyBullet 3D physics engine with GUI
- **Physics**: Realistic rigid body dynamics and collisions
- **Sensors**: 3D LiDAR ray casting in physics simulation
- **Robot Model**: URDF-based differential drive robot with wheels
- **Navigation**: Same A* and controller as Level 1, adapted for 3D physics

## Installation

### Prerequisites
- Python 3.11 or higher
- Conda (recommended) or venv

### Setup with Conda (Recommended)

```bash
# Create and activate environment for Level 1 (2D)
conda create -n level1_2d_sim python=3.14
conda activate level1_2d_sim
pip install -r requirements.txt

# Create and activate environment for Level 2 (3D)
conda create -n level1_3d_sim python=3.14
conda activate level1_3d_sim
pip install -r requirements.txt
```

### Dependencies
- `numpy` - Numerical computations
- `matplotlib` - 2D visualization
- `pybullet` - 3D physics simulation

## Usage

### Running Level 1 (2D Simulation)

```bash
conda activate level1_2d_sim
python3 level1_2d_sim/src/main.py
```

This will open a Matplotlib window showing:
- **Left panel**: Ground truth environment with robot, obstacles, LiDAR rays, and planned path
- **Right panel**: SLAM-built occupancy grid map

### Running Level 2 (3D Simulation)

```bash
conda activate level1_3d_sim
python3 level2_3d_sim/src/main.py
```

This will open a PyBullet GUI window showing the 3D warehouse environment with the robot navigating autonomously.

## Core Components

### Navigation
- **A* Planner**: Grid-based optimal path planning with 8-directional motion
- **Pure Pursuit Controller**: Path following with dynamic lookahead distance
- **Obstacle Avoidance**: Collision checking during planning

### Perception
- **LiDAR Simulation**: 360° scanning with configurable range and resolution
- **SLAM**: Simple occupancy grid mapping using ray tracing
- **Localization**: Odometry-based position tracking

### Robot Model
- **Differential Drive**: Two motorized wheels + caster wheel
- **Kinematics**: Standard unicycle model (v, ω)
- **Dimensions**: 0.4m × 0.3m × 0.15m (L × W × H)

## Algorithm Details

### A* Path Planning
- **Heuristic**: Euclidean distance to goal
- **Cost**: Actual distance traveled (1.0 for cardinal, √2 for diagonal)
- **Grid Resolution**: Configurable (1.0m for 2D, 0.5m for 3D)

### Pure Pursuit Control
- **Lookahead Distance**: 5.0m
- **Gains**: Kp_linear = 1.0, Kp_angular = 2.0
- **Velocity Limits**: Linear 0-5 m/s, Angular ±2 rad/s

### SLAM Mapping
- **Map Representation**: Occupancy grid (0.0 = free, 1.0 = occupied, 0.5 = unknown)
- **Update Method**: Incremental ray tracing with probability updates

## Future Enhancements

### Planned Features
- [ ] Vision-based object detection (YOLOv5/ViT)
- [ ] Barcode/QR code reading
- [ ] Multi-sensor fusion (LiDAR + Camera + IMU)
- [ ] ROS/ROS2 integration
- [ ] Gazebo simulation support
- [ ] Real hardware deployment (Raspberry Pi + Arduino)
- [ ] Advanced SLAM (Cartographer/RTAB-Map)
- [ ] Path optimization and replanning
- [ ] Multi-robot coordination

### Long-term Vision
Develop a complete warehouse automation solution capable of:
- Autonomous inventory scanning and tracking
- Product identification and localization
- Shelf-level navigation and scanning
- Integration with warehouse management systems

## Repository

```bash
git clone https://github.com/K-bhuvan/project1_warehouse_automation.git
cd project1_warehouse_automation
```

## Citations

### Libraries and Frameworks
```bibtex
@software{pybullet,
  title = {PyBullet, a Python module for physics simulation for games, robotics and machine learning},
  author = {Coumans, Erwin and Bai, Yunfei},
  url = {https://pybullet.org/},
  year = {2016--2023}
}

@article{numpy,
  title = {Array programming with NumPy},
  author = {Harris, Charles R. and Millman, K. Jarrod and van der Walt, Stéfan J. and others},
  journal = {Nature},
  volume = {585},
  pages = {357--362},
  year = {2020},
  doi = {10.1038/s41586-020-2649-2}
}

@article{matplotlib,
  title = {Matplotlib: A 2D Graphics Environment},
  author = {Hunter, John D.},
  journal = {Computing in Science \& Engineering},
  volume = {9},
  number = {3},
  pages = {90--95},
  year = {2007},
  doi = {10.1109/MCSE.2007.55}
}
```

### Algorithms
```bibtex
@article{astar,
  title = {A Formal Basis for the Heuristic Determination of Minimum Cost Paths},
  author = {Hart, Peter E. and Nilsson, Nils J. and Raphael, Bertram},
  journal = {IEEE Transactions on Systems Science and Cybernetics},
  volume = {4},
  number = {2},
  pages = {100--107},
  year = {1968},
  doi = {10.1109/TSSC.1968.300136}
}

@inproceedings{pure_pursuit,
  title = {Implementation of the Pure Pursuit Path Tracking Algorithm},
  author = {Coulter, R. Craig},
  booktitle = {Robotics Institute, Carnegie Mellon University},
  year = {1992},
  note = {Technical Report CMU-RI-TR-92-01}
}

@inproceedings{slam,
  title = {Simultaneous Localization and Mapping: Part I},
  author = {Durrant-Whyte, Hugh and Bailey, Tim},
  journal = {IEEE Robotics \& Automation Magazine},
  volume = {13},
  number = {2},
  pages = {99--110},
  year = {2006},
  doi = {10.1109/MRA.2006.1638022}
}
```

## License

MIT License

Copyright (c) 2025 K. Bhuvan Tej

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Acknowledgments

- **PyBullet** team for the excellent physics simulation framework
- **NumPy** and **Matplotlib** communities for foundational scientific computing tools
- Robotics research community for A*, Pure Pursuit, and SLAM algorithms
- Open-source robotics projects (ROS, TurtleBot) for inspiration

---

**Note**: This project is for educational and research purposes ONLY. Feel free to use, modify, and contribute!
