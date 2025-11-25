## Projec1: Warehouse Inventory Rover (Intermediate)

## Goal: 
Develop a small autonomous mobile robot(in simulation mode that be implemented in my laptop) that can map a warehouse aisle and perform inventory
scanning. The rover should navigate indoor warehouse corridors, build a map of its environment, and use
vision to identify products or read markers on shelves. This demonstrates autonomous navigation and basic semantic understanding useful for retail/warehouse robotics (e.g. scanning stock or locating items).

### Sensors & Inputs: 
The rover uses a 2D LiDAR or depth camera for SLAM (Simultaneous Localization and Mapping) and obstacle detection, plus an RGB camera for product recognition or barcode/QR code reading. An IMU and wheel encoders can be included for odometry to improve localization accuracy (sensor fusion with LiDAR) (For example, low-cost 360° LiDAR like RPLidar A1 and a webcam or Intel RealSense depth camera would suffice.)

### Core AI/ML Models: 
Implement a SLAM algorithm (such as ROS Gmapping or Cartographer) to concurrently build a map and localize the robot Use a deep learning vision model for semantic tasks – e.g. a YOLOv5 object detector to recognize product boxes or shelf markers in the camera feed This gives the robot an understanding of its environment (identifying specific inventory or tag locations). The state-of-the-art approach for mapping could incorporate multi-sensor SLAM (combining camera and LiDAR) to improve robustness. For inventory recognition, a modern Vision Transformer (ViT) based
classifier or OCR model could be used if scanning text labels.

### Physical Actuation: 
A wheeled mobile base (e.g. a differential-drive robot using Raspberry Pi + Arduino motor controller) carries the sensors. It should be capable of moving autonomously down aisles and stopping in front of shelves. No manipulator is required at this stage (the focus is on mobility and perception), but the robot could have a simple pan-tilt for the camera to scan different shelf levels. The platform could be a custom-built rover or a modified TurtleBot equipped with the chosen sensors.

### Reasoning Layer: 
Use the ROS Navigation stack (or a custom planner) for autonomous navigation and path planning along the aisle. The robot plans collision-free paths on the built map (e.g. using Dijkstra/A on a 2D occupancy grid) and employs a local planner for obstacle avoidance (like Dynamic Window Approach). A behavior could be scripted: e.g., patrol the aisle while stopping at pre-defined shelf locations to scan items. The reasoning also involves simple decision-making: if an item of interest is detected on a shelf, log its position. This layer ensures the robot not only localizes and maps* but also uses that map for goal-directed movement(e.g. going to shelf 5 upon command).