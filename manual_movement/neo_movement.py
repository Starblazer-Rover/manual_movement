import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray


class NeoSubscriber(Node):
    
    def __init__(self):
        super().__init__('neo_subscriber')
        self.subscription = self.create_subscription(Int32MultiArray, '/movement/Controller', self.listener_callback, 1)
        self.subscription

    def listener_callback(self, msg):
        