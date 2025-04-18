import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Twist

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'pharma_delivery', 'pharma_delivery')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config_04_2025 as conf

class InitPosition(Node):
    def __init__(self):
        super().__init__("init_pos")
        self.servo_publ = self.create_publisher(UInt8MultiArray, 'servo/to_write', 3)
        self.wheel_publ = self.create_publisher(Twist, '/cmd_vel', 3)
        self.timer = self.create_timer(1/3, self.publish)
        self.iter = 0
        self.idx = 0

        self.sequnce = [3,4]
    
    def publish(self):
        self.iter += 1
        if self.iter >= 5:
            self.idx += 1
            self.iter = 0
        if self.idx >= len(self.sequnce):
            self.idx = len(self.sequnce) - 1
        pos = conf.arm_positions[self.sequnce[0]][0]
        servo_msg = UInt8MultiArray()
        servo_msg.data = bytes(pos)
        self.servo_publ.publish(servo_msg)
        self.get_logger().info(f'sended {pos}')

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.wheel_publ.publish(twist_msg)
        self.get_logger().info(f"twist publed")


def main():
    rclpy.init()
    node = InitPosition()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()