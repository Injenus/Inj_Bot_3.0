import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from threading import Lock
import json
import copy

import sys, os
modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf


class StartFinish(Node):
    def __init__(self):
        super().__init__("start_finish")

        self.dt = conf.dt

        self.lidar_subs = self.create_subscription(String, 'lidar/obstacles', self.update_distances, 3)
        self.mode_subs = self.create_subscription(Int8, 'start_finish', self.update_mode, 3)

        self.publ_twist = self.create_publisher(Twist, '/cmd_vel', 3)

        self.timer = self.create_timer(self.dt, self.send_speed)

        self.mode = 0 # 0- стоп - -1 финиш 1- старт

        self.target_front_dist = conf.target_front_dist_start
        self.target_back_dist = conf.target_back_dist_finish
        self.lidar_lock = Lock()
        self.mode_lock = Lock()
        
        self.lidar_basic = {
                0: -1,
                90: -1,
                180: -1,
                270: -1
            }
        
        self.base_x_speed = conf.base_linear_x_speed

        self.can_stop = False


    def update_mode(self, msg):
        with self.mode_lock:
            if msg.data != self.mode:
                self.mode = msg.data
                self.cam_stop = True

    def update_distances(self, msg):
        data = json.loads(msg.data)
        data = {int(k): v for k, v in data.items()}

        with self.lidar_lock:
            self.lidar_basic = {
                0: data[0],
                90: data[90],
                180: data[180],
                270: data[270]
            }

        with self.mode_lock:
            mode = self.mode
            if any(not isinstance(value, float) or not isinstance(value, int) for value in self.lidar_basic.values()):
                self.mode = -2
                #self.get_logger().info(f"NOT NUMERIC LIDAR DATA")
                #raise ValueError(f'NOT NUMERIC LIDAR DATA {self.lidar_basic}')
            else:
                self.mode = mode
            if any(value < 0 for value in self.lidar_basic.values()):
                self.mode = -3
                #raise ValueError(f'NEGATIVE LIDAR DATA {self.lidar_basic}')
                #self.get_logger().info(f"NEGATIVE LIDAR DATA")
            else:
                self.mode = mode

    def send_speed(self):
        
        with self.lidar_lock:
            lidar_data = copy.deepcopy(self.lidar_basic)

        with self.mode_lock:    
            if self.mode == 0:
                if self.can_stop:
                    msg = Twist()
                    msg.linear.x = 0.0
                    self.publ_twist.publish(msg)
                    self.can_stop = False

            elif self.mode == 1:
                if lidar_data[0] > self.target_front_dist:
                    self.get_logger().info(f'sf {lidar_data[0]}', throttle_duration_sec=0.3)
                    msg = Twist()
                    msg.linear.x = self.base_x_speed
                    self.publ_twist.publish(msg)
                elif self.cam_stop:
                    msg = Twist()
                    msg.linear.x = 0.0
                    self.publ_twist.publish(msg)
                    self.can_stop = False
                    self.mode = 0

            elif self.mode == -1:
                if lidar_data[180] < self.target_back_dist:
                    self.get_logger().info(f'sf {lidar_data[180]}', throttle_duration_sec=0.3)
                    msg = Twist()
                    msg.linear.x = self.base_x_speed
                    self.publ_twist.publish(msg)
                elif self.cam_stop:
                    msg = Twist()
                    msg.linear.x = 0.0
                    self.publ_twist.publish(msg)
                    self.can_stop = False
                    self.mode = 0



def main(args=None):
    rclpy.init(args=args)
    node = StartFinish()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


