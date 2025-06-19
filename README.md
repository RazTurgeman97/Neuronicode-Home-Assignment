# ROS 2 Fast Sensor Fusion Mapper

This ROS 2 package provides a lightweight, high-performance node to fuse 128-beam LiDAR and RGB-D camera data into a minimal 2D world map. It is optimized for NVIDIA Jetson Orin platforms and designed for agile robotics applications requiring real-time situational awareness.

## Overview

The system generates a 2D occupancy grid representing **traversable ground paths**. It achieves this by:
1.  **LiDAR Ground Detection:** Filtering LiDAR points at near-zero height to reliably map the ground plane.
2.  **Camera Obstacle Detection:** Using the pretrained **MiDaS** depth estimation model on the camera feed to identify nearby obstacles.
3.  **Data Fusion:** Combining both sources to produce a final map where "free space" is defined as areas that are identified as ground by the LiDAR and *not* identified as an obstacle by the camera.

This approach ensures robust ground detection from the LiDAR while leveraging the camera's dense data to see obstacles the sparse LiDAR might miss.

## Features

-   **High Performance:** Exceeds 5 FPS on Jetson Orin NX by leveraging PyTorch with CUDA acceleration.
-   **ROS 2 Humble Integration:** Standard Python node structure for easy deployment.
-   **Minimalist Mapping:** Outputs a simple 2D `mono8` image, perfect as an input for navigation planners.
-   **Robust Fusion Logic:** Combines the strengths of LiDAR (geometry) and camera (dense obstacles).

## Installation

### 1. Prerequisites
-   Ubuntu 22.04 with ROS 2 Humble.
-   NVIDIA Jetson with JetPack 5+ installed.
-   PyTorch for Jetson installed.
-   Intel RealSense SDK and `realsense-ros` wrapper.
-   LiDAR driver publishing a `sensor_msgs/PointCloud2` message.

### 2. Dependencies
Install necessary Python libraries and ROS packages.
```bash
# System dependencies
sudo apt update
sudo apt install -y python3-pip ros-humble-cv-bridge


# Python packages
pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/l4t
pip3 install numpy opencv-python
```

3. Build the ROS 2 Package

# 1. Navigate to your ROS 2 workspace source directory
cd ~/ros2_ws/src/

# 2. Clone the repository
git clone https://github.com/robotics-expert/ros2-fast-fusion-mapper.git

# 3. Build the package
cd ~/ros2_ws/
colcon build --packages-select fast_fusion_mapper

# 4. Source the workspace
source install/setup.bash

Usage

Launch the node using the provided launch file or directly. Ensure your LiDAR and camera drivers are running and publishing to the correct topics.

# Source your workspace
source install/setup.bash

# Run the node
ros2 run fast_fusion_mapper fusion_mapper_node

Required Topics

    /lidar/points (sensor_msgs/PointCloud2): Input from the 128-beam LiDAR.
    /camera/color/image_raw (sensor_msgs/Image): Input from the RealSense camera.

Published Topic

    /world_map (sensor_msgs/Image): Output 2D map. White pixels represent free, traversable space.

Real-World Project Recommendations
Limitations of This Implementation

    No Odometry: The map is stateless and re-generated from scratch each frame. It does not stitch maps over time or account for robot movement.
    Flat World Assumption: The ground filter (z < 0.15m) assumes the robot is on a relatively flat plane.
    No Camera/LiDAR Calibration: The fusion is done in separate data spaces and combined with a logical AND. It does not use a calibrated extrinsic transformation (TF) between the camera and LiDAR, which would be more accurate.

Recommendations for Production

    Integrate with SLAM: Feed the robot's pose from a SLAM system (like slam_toolbox or rtabmap_ros) to build a persistent, global map instead of a transient, local one.
    Use TF2: For a more accurate fusion, use the tf2 library to project LiDAR points into the camera's image plane or re-project camera pixels into the 3D world. This requires a precise extrinsic calibration between the sensors.
    TensorRT Optimization: For maximum performance on Jetson, convert the PyTorch model to a TensorRT engine. This can provide a 2-3x speedup over native PyTorch with CUDA.
    Parameterize Hardcoded Values: Convert values like the ground height threshold (0.15), map size, and topic names into ROS 2 parameters for easier configuration.

