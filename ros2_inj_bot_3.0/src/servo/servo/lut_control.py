import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import String
import os
import sys

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'servo', 'servo')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config_servo as conf

class LUTcontrol(Node):
    def __init__(self):
        super().__init__('lut_control')

        self.subscriber = self.create_subscription(UInt8, 'servo/lut', self.lut_callback, 3)
        self.publisher = self.create_publisher(UInt8MultiArray, 'servo/to_write', 3)

        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)
        msg = String(data = self.get_name())
        self.ready_pub.publish(msg)

    def lut_callback(self, msg):
        idx = msg.data
        if idx in conf.arm_positions:
            pos = conf.arm_positions[idx][0]
            servo_msg = UInt8MultiArray()
            servo_msg.data = bytes(pos)
            self.publisher.publish(servo_msg)
            self.get_logger().info(f'lut to servo {servo_msg.data}')
        else:
            self.get_logger().info(f'Non existent command')

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init(args=None)
    lut = LUTcontrol()
    rclpy.spin(lut)
    lut.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



        