#!/home/raz/projects/Neuronicode-Home-Assignment/ros2_ws/venv/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO

class AwarenessNode(Node):
    def __init__(self):
        super().__init__('awareness_node')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n-seg.pt')  # pretrained model
        self.grid_pub = self.create_publisher(OccupancyGrid, 'awareness_map', 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        self.create_subscription(PointCloud2, '/scan', self.lidar_cb, 10)
        self.latest_mask = None
        self.grid_size, self.resolution = 200, 0.05  # 10x10m area, 5cm cells
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(img, imgsz=320, conf=0.5, verbose=False)[0]
        mask = results.masks.xy if results.masks else None
        self.latest_mask = self.generate_mask(mask, img.shape)

    def lidar_cb(self, msg):
        if self.latest_mask is None:
            return
        self.update_grid()
        self.publish_grid()

    def generate_mask(self, mask_pts, shape):
        mask_img = np.zeros(shape[:2], dtype=np.uint8)
        if mask_pts:
            for pts in mask_pts:
                cv2.fillPoly(mask_img, [np.array(pts, dtype=np.float32)], (255,))
        return mask_img

    def update_grid(self):
        self.grid.fill(0)  # assume free space
        if self.latest_mask is not None:
            mask_resized = cv2.resize(self.latest_mask, (self.grid_size, self.grid_size))
            self.grid[mask_resized > 0] = 100  # mark obstacles

    def publish_grid(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'base_link'
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = grid_msg.info.height = self.grid_size
        grid_msg.info.origin.position.x = grid_msg.info.origin.position.y = -5.0
        grid_msg.data = self.grid.flatten().tolist()
        self.grid_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AwarenessNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
