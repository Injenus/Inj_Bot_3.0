import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import json

class BorderMove(Node):
    def __init__(self):
        super().__init__("border_move")

        self.mode_subs = self.create_subscription(UInt8, 'border_move', self.update_mode, 3)
        self.lidar_subs = self.create_subscription(String, 'lidar/obstacles', self.update_distances, 3)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 3)

        self.timer = self.create_timer(0.005, self.send_speed)
        
        self.mode = 0 # 0 - stop, 1 -move
        self.lidar_r = 0.025
        self.target_border_dist = 0.28 + self.lidar_r # m
        self.front_turn_dist = 0.16 + self.lidar_r

        self.lidar_lock = False
        self.lidar_basic = {
                0: -1,
                90: -1,
                180: -1,
                270: -1
            }
        
        self.base_x_speed = 0.12
        self.max_abs_z_speed = 4.0

        
    def update_mode(self, msg):
        self.mode = msg.data

    def update_distances(self, msg):
        if not self.lidar_lock:
            data = json.loads(msg.data)
            data = {int(k): v for k, v in data.items()}

            if not all(value != -1 for value in self.lidar_basic.values()): ## TODO КОСТЛ ДЛЯ ДЕБАГА!!! УБРАТЬ!!!
                if self.mode == 0:
                    self.mode = 1

            self.lidar_basic = {
                0: data[0],
                90: data[90],
                180: data[180],
                270: data[270]
            }

    def pid_side(self, err):
        Kp = 25.0
        control = Kp * err
        return max(min(control, self.max_abs_z_speed), -self.max_abs_z_speed)

    def pid_front(self, err):
        Kp = 50.0
        control = Kp * err
        return max(min(control, self.max_abs_z_speed), 0.0)

    def send_speed(self):
        msg = Twist()
        if self.mode:
            ang_w = 0.0
            self.lidar_lock = True
            

            side_error = self.target_border_dist - self.lidar_basic[90]
            ang_w = self.pid_side(side_error)
            self.get_logger().info(f'side err {side_error}, {ang_w}')

            if self.lidar_basic[0] < self.front_turn_dist:
                ang_w = abs(self.pid_front(self.front_turn_dist - self.lidar_basic[0])) # always >0 because move by ccw
                self.get_logger().info(f'font err {self.front_turn_dist - self.lidar_basic[0]}, {ang_w}')

            msg.linear.x = self.base_x_speed
            msg.angular.z = ang_w

            self.lidar_lock = False
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = BorderMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()