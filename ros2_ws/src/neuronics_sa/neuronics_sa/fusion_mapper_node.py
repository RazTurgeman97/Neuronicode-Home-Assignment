#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import torch
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

class FusionMapperNode(Node):
    def __init__(self):
        super().__init__('fusion_mapper_node')
        # Initialize parameters & models
        self.bridge = CvBridge()
        self.model = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', trust_repo=True).to('cuda').eval()
        self.lidar_pcd = None
        self.map_size = (500, 500) # 50m x 50m at 10cm/pixel
        self.map_center = (self.map_size[0] // 2, self.map_size[1] // 2)

        # ROS Subscribers
        self.create_subscription(PointCloud2, '/lidar/points', self.lidar_callback, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        
        # Publisher for the final map (optional, good practice)
        self.map_publisher = self.create_publisher(Image, '/world_map', 10)
        self.get_logger().info("Fusion Mapper Node Initialized")

    def lidar_callback(self, msg):
        # Store latest LiDAR point cloud, keeping only x, y, z fields
        gen = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        self.lidar_pcd = np.array(list(gen))

    def image_callback(self, msg):
        if self.lidar_pcd is None:
            return
        
        # 1. Generate Obstacle Mask from Camera using MiDaS
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img_tensor = torch.from_numpy(cv2.resize(cv_image, (256, 256)).transpose(2,0,1)).float().unsqueeze(0).to('cuda')
        with torch.no_grad():
            depth_pred = self.model(img_tensor).squeeze().cpu().numpy()
        
        # Create obstacle mask: low depth values = close obstacles
        obstacle_mask = (depth_pred > np.percentile(depth_pred, 75)).astype(np.uint8) * 255
        obstacle_mask_resized = cv2.resize(obstacle_mask, self.map_size)

        # 2. Generate Ground Plane from LiDAR
        ground_map = np.zeros(self.map_size, dtype=np.uint8)
        ground_points = self.lidar_pcd[np.abs(self.lidar_pcd[:, 2]) < 0.15] # Filter for ground points
        
        for p in ground_points:
            u = int(self.map_center[0] - p[1] * 10) # X in map is -Y in LiDAR
            v = int(self.map_center[1] - p[0] * 10) # Y in map is -X in LiDAR
            if 0 <= u < self.map_size[0] and 0 <= v < self.map_size[1]:
                ground_map[v, u] = 255

        # 3. Fuse Maps: Free space = Ground (from LiDAR) AND NOT Obstacle (from Camera)
        free_space_map = cv2.bitwise_and(ground_map, cv2.bitwise_not(obstacle_mask_resized))

        # Publish and visualize the result
        self.map_publisher.publish(self.bridge.cv2_to_imgmsg(free_space_map, "mono8"))
        # cv2.imshow("Fused Map", free_space_map); cv2.waitKey(1) # Uncomment for local debug

def main(args=None):
    rclpy.init(args=args)
    node = FusionMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
