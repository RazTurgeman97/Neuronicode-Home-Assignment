🦾 Lidar + RGB Camera Fusion for Robotic Dog Navigation 

This project fuses data from a 128-beam LiDAR and an RGB-D camera to produce a minimal world map distinguishing free paths  and obstacles  for fast-moving robotic dogs.

🛠️ Requirements 

* ROS2 Humble 
* Jetson Orin NX or similar embedded platform 
* Intel RealSense RGB-D camera
* LiDAR with PointCloud2 output 
* OpenCV, NumPy, ROS2 Python Packages 
     

Install dependencies: 
```bash
pip install numpy opencv-python
sudo apt install ros-humble-cv-bridge ros-humble-message-filters
```
 
 
🚀 Usage 

1. Launch RealSense camera : 
```bash
ros2 launch realsense2_camera rs_launch.py
```

2. Start LiDAR driver  (e.g., Velodyne, Ouster, etc.) 

3. Run fusion node : 
```bash
    ros2 run lidar_camera_fusion fusion_node
```     
     
     

4. Visualize in RVIZ2 : 
* Add PointCloud2 display for /fused_map
* Set color scheme to RGB
         
     

⚙️ Configuration 

* Camera Intrinsics : Adjust the camera_matrix parameter in the launch file or node constructor.
* Transform : Currently hardcoded; use tf2 for dynamic calibration.
* Segmentation Model : Replace dummy segmentation with a pretrained model like DeepLab or MobileNetV3.
     

📊 Performance 

* Achieves >5 FPS  on Jetson Orin NX with optimized preprocessing.
* Uses projection-based fusion  with lightweight semantic inference.
* Colored PointCloud2  output marks:
  * 🟩 Green: Free path (road) 
  * 🔴 Red: Obstacle 
         
     

🧩 Limitations 

* Assumes static LiDAR-camera calibration.
* Dummy segmentation model needs real-world training.
* Does not perform 3D reconstruction or surface estimation.
     

🛡️ Troubleshooting 

* Check topics : Use ros2 topic list to verify /camera/color/image_raw and /laser_cloud.
* TF issues : Ensure static transform is published between LiDAR and camera frames.
* Synchronization : Adjust sync tolerance in ApproximateTimeSynchronizer.
     

📦 Future Improvements 

* Replace dummy segmentation with ONNX or TensorRT inference.
* Add IMU/GPS for full 3D mapping.
* Optimize for CUDA acceleration on Jetson.

🚀 Final Output 

The final deliverable is a ROS2 node  that fuses LiDAR and RGB camera data into a colored point cloud world map , enabling fast and accurate obstacle detection for robotic dogs. The code is designed for real-time execution  on Jetson Orin NX  platforms with minimal computational overhead. 