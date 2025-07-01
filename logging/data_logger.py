import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime
import csv

class SensorDataLogger(Node):
    def __init__(self):
        super().__init__('sensor_data_logger')

        # 이미지 저장할 폴더 만들기
        self.img_dir = 'images'
        os.makedirs(self.img_dir, exist_ok=True)

        # CSV 파일 (ODOM 데이터 저장)
        self.csv_file = 'odom_data.csv'
        # CSV 헤더 쓸 때는 파일 없으면 새로 만들기
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'pos_x', 'pos_y', 'pos_z', 'orient_x', 'orient_y', 'orient_z', 'orient_w',
                                 'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'])

        self.bridge = CvBridge()

        # 토픽 구독
        self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/depth_camera/depth/image_raw', self.depth_image_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            img_path = os.path.join(self.img_dir, f'rgb_{timestamp}.png')
            cv2.imwrite(img_path, cv_img)
            self.get_logger().info(f'Saved RGB image: {img_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save RGB image: {e}')

    def depth_image_callback(self, msg):
        try:
            # depth는 보통 16UC1 or 32FC1, 그대로 저장하면 크기 큼.
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            img_path = os.path.join(self.img_dir, f'depth_{timestamp}.png')
            cv2.imwrite(img_path, cv_img)
            self.get_logger().info(f'Saved Depth image: {img_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save Depth image: {e}')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                pos.x, pos.y, pos.z,
                orient.x, orient.y, orient.z, orient.w,
                linear.x, linear.y, linear.z,
                angular.x, angular.y, angular.z
            ])
        self.get_logger().info(f'Logged odom at {timestamp}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

