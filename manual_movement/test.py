import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class TestNode(Node):
    
    def __init__(self):
        super().__init__('test_node')

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/odom/target_point', self.pose_callback, 10)

    def odom_callback(self, msg):
        print('odom')

    def pose_callback(self, msg):
        print('pose')


def main(args=None):
    rclpy.init(args=args)

    node = TestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()