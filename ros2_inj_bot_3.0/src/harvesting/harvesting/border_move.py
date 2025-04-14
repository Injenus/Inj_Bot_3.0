import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from threading import Lock
import json
import copy

class PID():
    def __init__(self, p,i,d, min_v, max_v, dt, max_integral=-1):
        self.p = p
        self.i = i
        self.d = d
        self.min_v = min_v
        self.max_v = max_v
        self.err = 0
        self.prev_err = 0
        self.integral = 0
        if max_integral == -1:
            self.max_integral = max_v/4
        else:
            self.max_integral = max_integral
        self.dt = dt

    def calculate(self, err):
        self.err = err
        P = self.p*self.err
        I = max(min(self.i*(self.integral + self.err*self.dt, self.max_integral)), -self.max_integral)
        D = self.d*(self.err - self.prev_err)/ self.dt
        self.prev_err = self.err
        return  max(min(P+I+D, self.max_v), self.min_v)
    


class BorderMove(Node):
    def __init__(self):
        super().__init__("border_move")

        self.dt = 0.005
        self.mode_subs = self.create_subscription(UInt8, 'border_move', self.update_mode, 3)
        self.lidar_subs = self.create_subscription(String, 'lidar/obstacles', self.update_distances, 3)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 3)

        self.timer = self.create_timer(self.dt, self.send_speed)
        
        self.mode = 0 # 0 - stop, 1 -move
        self.lidar_r = 0.025
        self.target_border_dist = 0.28 + self.lidar_r # m
        self.front_turn_dist = 0.25 + self.lidar_r

        self.lidar_lock = Lock()
        self.mode_lock = Lock()
        self.lidar_basic = {
                0: -1,
                90: -1,
                180: -1,
                270: -1
            }
        
        self.base_x_speed = 0.12
        self.max_abs_z_speed = 4.0

        self.pid_side = PID(25, 0, 0, -self.max_abs_z_speed, self.max_abs_z_speed, self.dt)
        self.pid_front = PID(20, 0, 0, 0, self.max_abs_z_speed, self.dt)


        
    def update_mode(self, msg):
        with self.mode_lock:
            self.mode = msg.data

    def update_distances(self, msg):
        with self.lidar_lock:
            data = json.loads(msg.data)
            data = {int(k): v for k, v in data.items()}

            #
            if any(value == -1 for value in self.lidar_basic.values()): ## TODO КОСТЫЛЬ ДЛЯ ДЕБАГА!!! УБРАТЬ!!!
                if self.mode == 0:
                    self.mode = 1
            #

            self.lidar_basic = {
                0: data[0],
                90: data[90],
                180: data[180],
                270: data[270]
            }
            if any(not isinstance(value, float) or not isinstance(value, int) for value in self.lidar_basic.values()):
                raise ValueError(f'NOT NUMERIC LIDAR DATA {self.lidar_basic}')
            if any(value < 0 for value in self.lidar_basic.values()):
                raise ValueError(f'NEGATIVE LIDAR DATA {self.lidar_basic}')


    def send_speed(self):
        with self.mode_lock:
            current_mode = self.mode
        with self.lidar_lock:
            lidar_data = copy.deepcopy(self.lidar_basic)

        msg = Twist()

        if current_mode:
            ang_w = 0.0

            side_error = self.target_border_dist - lidar_data[90]
            ang_w = self.pid_side.calculate(side_error)
            self.get_logger().info(f'side err {side_error}, {ang_w}')

            if lidar_data[0] < self.front_turn_dist:
                ang_w = self.pid_front.calculate(self.front_turn_dist - lidar_data[0]) # always >0 because move by ccw
                self.get_logger().info(f'font err {self.front_turn_dist - lidar_data[0]}, {ang_w}')

            msg.linear.x = self.base_x_speed
            msg.angular.z = ang_w

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