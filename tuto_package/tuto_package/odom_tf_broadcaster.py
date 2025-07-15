import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry,          # 메시지 타입
            '/odom',           # Odometry 토픽 이름 (본인 환경에 따라 수정)
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        # Odometry 메시지에서 tf 변환으로 필요한 정보 추출
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id      # 보통 'odom'
        t.child_frame_id = 'link_0'        # 보통 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
