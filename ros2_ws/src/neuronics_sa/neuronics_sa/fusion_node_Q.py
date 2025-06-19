import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
# import sensor_msgs.msg._point_cloud2 as pc2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
import message_filters
from std_msgs.msg import Header

class FusionMapper(Node):
    def __init__(self):
        super().__init__('fusion_mapper')
        self.declare_parameter('camera_matrix', [600, 0, 320, 0, 600, 240, 0, 0, 1])
        self.K = np.array(self.get_parameter('camera_matrix').get_parameter_value().integer_array_value).reshape(3,3)
        self.bridge = CvBridge()
        # Subscribe to synchronized RGB and LiDAR
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/laser_cloud')
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.lidar_sub], 10, 0.1)
        self.sync.registerCallback(self.process)
        # Publisher
        self.map_pub = self.create_publisher(PointCloud2, '/fused_map', 10)
        # Dummy segmentation model
        self.segment = lambda img: cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) > 128

    def process(self, rgb_msg, lidar_msg):
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        mask = self.segment(cv_image)
        header = Header(frame_id=lidar_msg.header.frame_id)
        points = []
        for p in pc2.read_points(lidar_msg, field_names=["x", "y", "z"], skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            if z <= 0: continue
            u = int((self.K[0,0]*x/z + self.K[0,2]))
            v = int((self.K[1,1]*y/z + self.K[1,2]))
            if 0 <= u < 640 and 0 <= v < 480:
                color = (0, 255, 0) if mask[v, u] else (0, 0, 255)
            else:
                color = (255, 0, 0)
            points.append((x, y, z, *color))
        cloud = pc2.create_cloud_xyz32(header, [(p[0], p[1], p[2]) for p in points])
        self.map_pub.publish(cloud)
def main():
    rclpy.init()
    node = FusionMapper()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()