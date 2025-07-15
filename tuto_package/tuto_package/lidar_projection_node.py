import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class LidarProjectionNode(Node):
    def __init__(self):
        super().__init__('lidar_projection_node')
        
        self.bridge = CvBridge()
        
        #(Subscriber)
        self.image_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar/scan', self.lidar_callback, 10)
        
        # (Publisher)
        self.projection_pub = self.create_publisher(Image, '/projection/image_result', 10)
        
        # 데이터 저장 변수
        self.latest_image = None
        self.lidar_points = None
        
        
        # 1. 내부 파라미터: 카메라 행렬 (K)
        self.camera_matrix = np.array([
            [277.19135641132203, 0.0, 160.5],
            [0.0, 277.19135641132203, 120.5],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # 2. 내부 파라미터: 왜곡 계수 (D) - 시뮬레이션에서는 0으로 가정
        self.dist_coeffs = np.zeros(5, dtype=np.float64)

        # "로봇 -x(forward) <-> 카메라 z(forward)" 변환 반영
        R = np.array([
            [0, -1, 0],
            [0, 0, 1],
            [1, 0, 0]
        ])



        t = np.array([0., 0., 0.038])  # 라이다-카메라 상대 위치(URDF 기준)
        
        self.lidar_to_cam_transform = np.eye(4)
        self.lidar_to_cam_transform[:3, :3] = R
        self.lidar_to_cam_transform[:3, 3] = t

        self.R_lidar_to_cam = self.lidar_to_cam_transform[:3, :3]
        self.t_lidar_to_cam = self.lidar_to_cam_transform[:3, 3]

        self.get_logger().info('Lidar Projection Node has been started with hardcoded parameters.')

    def image_callback(self, msg):
        # 이미지가 들어오면 저장하고, 마지막으로 수신한 라이다 데이터로 투영 시도
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.lidar_points is not None:
            self.project_and_publish()

    def lidar_callback(self, msg):
        # 라이다 데이터가 들어오면 저장 (넘파이 배열로 변환)
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append(point)
        self.lidar_points = np.array(points)

    def project_and_publish(self):
        if self.latest_image is None or self.lidar_points is None:
            return

        img = self.latest_image.copy()
        
        # 모든 라이다 포인트를 카메라 좌표계로 일괄 변환 (행렬 연산)
        # (N, 3) @ (3, 3) -> (N, 3)
        cam_points = (self.R_lidar_to_cam @ self.lidar_points.T).T + self.t_lidar_to_cam

        # 카메라 앞쪽 (z > 0) 에 있는 포인트들만 필터링
        front_points = cam_points[cam_points[:, 2] > 0]

        print(f"front_points shape: {front_points.shape}")
        
        if front_points.shape[0] > 0:
            # 3D 포인트를 2D 이미지 평면에 일괄 투영
            image_points, _ = cv2.projectPoints(front_points, np.zeros(3), np.zeros(3), self.camera_matrix, self.dist_coeffs)
            
            # 이미지 경계 안에 있는 점들만 그리기
            h, w, _ = img.shape
            min_depth = 0.01
            max_depth = 10

            for i, p in enumerate(image_points):
                u, v = int(p[0][0]), int(p[0][1])
                if 0 <= u < w and 0 <= v < h:
                    depth = front_points[i, 2]
                    if (depth!=0) and (depth < min_depth):
                        norm_depth = 0.0
                    elif (depth==0) or (depth > max_depth):
                        norm_depth = 1.0
                    else:
                        norm_depth = (depth - min_depth) / (max_depth - min_depth)
                    intensity = (1.0 - norm_depth) * 255
                    color = cv2.applyColorMap(np.array([[int(intensity)]], dtype=np.uint8), cv2.COLORMAP_JET)[0][0].tolist()
                    cv2.circle(img, (u, v), 2, color, -1)

        # 결과 이미지 발행
        try:
            self.projection_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except Exception as e:
            self.get_logger().error(f'Failed to publish projection image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()