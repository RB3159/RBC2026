import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import os
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.bridge = CvBridge()

        # 이미지 저장 폴더 생성
        self.image_dir = 'images'
        os.makedirs(self.image_dir, exist_ok=True)

        # 로그 파일 생성
        self.log_file = open('robot_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Time', 'Position_X', 'Position_Y', 'Linear_Velocity', 'Angular_Velocity'])

        # sub 설정
        self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def image_callback(self, msg):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            image_path = os.path.join(self.image_dir, f'{timestamp}.png')
            cv2.imwrite(image_path, cv_image)
            self.get_logger().info(f'Image saved: {image_path}')
        except Exception as e:
            self.get_logger().error(f'Image save failed: {e}')

    def odom_callback(self, msg):
        time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        lin_vel = msg.twist.twist.linear.x
        ang_vel = msg.twist.twist.angular.z

        self.csv_writer.writerow([time, pos_x, pos_y, lin_vel, ang_vel])
        self.get_logger().info(f'Logged odometry: x={pos_x:.2f}, y={pos_y:.2f}, v={lin_vel:.2f}')

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
