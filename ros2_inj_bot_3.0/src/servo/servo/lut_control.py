import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt8


class LUTcontrol(Node):
    def __init__(self):
        super().__init__('lut_control')

        self.subscriber = self.create_subscription(UInt8, 'servo/lut', self.lut_callback, 3)
        self.publisher = self.create_publisher(UInt8MultiArray, 'servo/to_write', 3)

        