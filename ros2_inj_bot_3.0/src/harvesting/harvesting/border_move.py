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
        self.integral = max(min(self.integral + self.err*self.dt, self.max_integral), -self.max_integral)
        I = self.i * self.integral
        D = self.d*(self.err - self.prev_err)/ self.dt
        self.prev_err = self.err
        return  max(min(P+I+D, self.max_v), self.min_v)
    

class BorderMove(Node):
    def __init__(self):
        super().__init__("border_move")

        self.dt = conf.dt
        self.mode_subs = self.create_subscription(Int8, 'border_mode', self.update_mode, 3)
        self.lidar_subs = self.create_subscription(String, 'lidar/obstacles', self.update_distances, 3)
        self.publ_twist = self.create_publisher(Twist, '/cmd_vel', 3)


        self.timer = self.create_timer(self.dt, self.send_speed)
        
        self.mode = 0 # 0 - stop, 1 -move, -1 - reverse
        self.lidar_r = conf.lidar_r
        self.target_border_dist = conf.target_border_dist + self.lidar_r # m
        self.front_turn_dist = conf.front_turn_dist + self.lidar_r

        self.lidar_lock = Lock()
        self.mode_lock = Lock()
        
        self.lidar_basic = {
                0: -1,
                90: -1,
                180: -1,
                270: -1
            }
        
        self.base_x_speed = conf.base_linear_x_speed
        self.max_abs_z_speed = conf.max_abs_angular_w_speed
        self.base_w_speed = conf.base_angular_w_speed

        self.pid_side = PID(conf.side_p, conf.side_i, conf.side_d, -self.max_abs_z_speed, self.max_abs_z_speed, self.dt)
        self.pid_front = PID(conf.front_p, conf.front_i, conf.front_d, conf.min_abs_angular_w_speed, self.max_abs_z_speed, self.dt)
        
        # self.autotuner = AutoTuner([25.0, 0.0, 0.0], self.dt)
        # self.total_time = 0.0
        # self.max_tuning_time = 42.0  # Максимальное время настройки (30 сек)

        # with open('errs_pid.txt', 'a') as f:
        #     f.write('\n\n')

        self.can_stop = False

    
    def update_mode(self, msg):
        with self.mode_lock:
            self.mode = msg.data

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
        with self.mode_lock:
            current_mode = self.mode
        with self.lidar_lock:
            lidar_data = copy.deepcopy(self.lidar_basic)

        

        if current_mode:
            self.can_stop = True
            msg = Twist()
            ang_w = 0.0
            lin_x = 0.0 

            side_error = self.target_border_dist - lidar_data[90]
            front_error = self.front_turn_dist - lidar_data[0] if lidar_data[0] != -1 else 0

            ang_w = self.pid_side.calculate(side_error)
            #ang_w = (side_error/abs(side_error))*(1+side_error)**2
            self.get_logger().info(f'side {side_error}, {ang_w}', throttle_duration_sec=0.2)
            # with open('errs_pid.txt', 'a') as f:
            #     f.write(f'side {side_error}, {ang_w}\n')
            lin_x = self.base_x_speed * current_mode
            if abs(side_error) > conf.tolerance_side_error:
                lin_x /= 5

            if current_mode == 1:
                if lidar_data[0] < self.front_turn_dist:
                    #ang_w = self.pid_front.calculate(front_error)
                    ang_w = self.base_w_speed
                    #self.get_logger().info(f'font {front_error}, {ang_w}')
                    # with open('errs_pid.txt', 'a') as f:
                    #     f.write(f'font {front_error}, {ang_w}\n')

            msg.linear.x = lin_x
            msg.angular.z = ang_w

            self.publ_twist.publish(msg)
            self.get_logger().info(f'tw {msg.linear.x} {msg.angular.z}', throttle_duration_sec=0.3)
        elif self.can_stop:
            self.publ_twist.publish(Twist())
            self.can_stop = False

def main(args=None):
    rclpy.init(args=args)
    node = BorderMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()