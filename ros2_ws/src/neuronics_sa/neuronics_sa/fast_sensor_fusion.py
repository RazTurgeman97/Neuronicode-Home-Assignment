#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import torch
import cv2
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from threading import Lock
import time

class FastSensorFusion(Node):
    def __init__(self):
        super().__init__('fast_sensor_fusion')
        self.bridge = CvBridge()
        self.data_lock = Lock()
        
        # Map parameters optimized for speed
        self.map_res = 0.1  # 10cm resolution
        self.map_size = 100  # 10m x 10m
        self.map_origin = -5.0
        self.occupancy_grid = np.full((self.map_size, self.map_size), 50, dtype=np.int8)
        
        # GPU acceleration setup
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Load lightweight segmentation model
        self.model = torch.hub.load('pytorch/vision:v0.15.2', 'deeplabv3_mobilenet_v3_large', 
                                   weights='COCO_WITH_VOC_LABELS_V1').to(self.device).eval()
        
        # Camera intrinsics (Intel RealSense D435)
        self.fx, self.fy = 615.0, 615.0
        self.cx, self.cy = 320.0, 240.0
        
        # Data buffers
        self.latest_rgb = None
        self.latest_depth = None
        self.last_process_time = time.time()
        
        # ROS2 setup
        self.lidar_sub = self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_cb, 1)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_cb, 1)  
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/fused_occupancy_map', 1)
        self.timer = self.create_timer(0.15, self.publish_map)  # 6.7 FPS
        
    def lidar_cb(self, msg):
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True)])
        if len(points) == 0: return
        
        # Filter ground and project to grid
        ground_points = points[(points[:, 2] > -0.5) & (points[:, 2] < 0.3)]
        grid_x = np.clip(((ground_points[:, 0] - self.map_origin) / self.map_res).astype(int), 0, self.map_size-1)
        grid_y = np.clip(((ground_points[:, 1] - self.map_origin) / self.map_res).astype(int), 0, self.map_size-1)
        
        with self.data_lock:
            self.occupancy_grid[grid_y, grid_x] = 0  # Free space
            
        # Mark obstacles (above ground)
        obs_points = points[points[:, 2] > 0.3]
        if len(obs_points) > 0:
            obs_x = np.clip(((obs_points[:, 0] - self.map_origin) / self.map_res).astype(int), 0, self.map_size-1)
            obs_y = np.clip(((obs_points[:, 1] - self.map_origin) / self.map_res).astype(int), 0, self.map_size-1)
            with self.data_lock:
                self.occupancy_grid[obs_y, obs_x] = 100  # Occupied
                
    def rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        
    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        if self.latest_rgb is not None and time.time() - self.last_process_time > 0.1:
            self.process_rgbd()
            self.last_process_time = time.time()
            
    def process_rgbd(self):
        # Fast semantic segmentation
        rgb_small = cv2.resize(self.latest_rgb, (320, 240))
        input_batch = torch.from_numpy(rgb_small).permute(2,0,1).float().unsqueeze(0).to(self.device) / 255.0
        
        with torch.no_grad():
            output = self.model(input_batch)['out'][0].argmax(0).cpu().numpy()
            
        # Create road/obstacle masks (road=7, sidewalk=8, car=2, person=0)
        road_mask = np.isin(output, [7, 8]).astype(np.uint8)
        obstacle_mask = np.isin(output, [0, 2, 5, 6]).astype(np.uint8)
        
        # Project to world coordinates
        depth_small = cv2.resize(self.latest_depth, (320, 240)) / 1000.0
        v, u = np.mgrid[0:240:4, 0:320:4]
        
        valid = (depth_small[v, u] > 0.5) & (depth_small[v, u] < 8.0)
        z = depth_small[v, u][valid]
        x = (u[valid] - self.cx/2) * z / (self.fx/2)
        y = z
        
        # Map to occupancy grid
        grid_x = np.clip(((x - self.map_origin) / self.map_res).astype(int), 0, self.map_size-1)
        grid_y = np.clip(((y - self.map_origin) / self.map_res).astype(int), 0, self.map_size-1)
        
        road_vals = road_mask[v[valid]//4, u[valid]//4]
        obs_vals = obstacle_mask[v[valid]//4, u[valid]//4]
        
        with self.data_lock:
            self.occupancy_grid[grid_y[road_vals > 0], grid_x[road_vals > 0]] = 0
            self.occupancy_grid[grid_y[obs_vals > 0], grid_x[obs_vals > 0]] = 100
            
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.info.resolution = self.map_res
        msg.info.width = msg.info.height = self.map_size
        msg.info.origin.position.x = msg.info.origin.position.y = self.map_origin
        
        with self.data_lock:
            msg.data = self.occupancy_grid.flatten().astype(int).tolist()
        self.map_pub.publish(msg)

def main():
    rclpy.init()
    node = FastSensorFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
