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
        self.integral = max(min(self.integral + self.err*self.dt, self.max_integral), -self.max_integral)
        I = self.i * self.integral
        D = self.d*(self.err - self.prev_err)/ self.dt
        self.prev_err = self.err
        return  max(min(P+I+D, self.max_v), self.min_v)
    

class AutoTuner:
    def __init__(self, initial_params, dt):
        self.params = copy.deepcopy(initial_params)
        self.dt = dt
        self.tuning_mode = True
        self.stage = 0  # 0-P, 1-I, 2-D
        self.error_history = []
        self.accumulated_error = 0.0
        self.best_error = float('inf')
        self.best_params = copy.deepcopy(initial_params)
        self.tuning_time = 0.0  # Заменяем расстояние на время
        self.tuning_interval = 2.0  # Интервал настройки в секундах
        self.delta = [0.5, 0.01, 0.01]  # шаги для P, I, D

    def update_error(self, error):
        if not self.tuning_mode:
            return

        self.accumulated_error += abs(error) * self.dt
        self.tuning_time += self.dt  # Накопление времени

        if self.tuning_time >= self.tuning_interval:
            self.adjust_params()
            self.tuning_time = 0.0

    def adjust_params(self):
        if self.accumulated_error < self.best_error:
            self.best_error = self.accumulated_error
            self.best_params = copy.deepcopy(self.params)
            self.delta[self.stage] *= 1.1
        else:
            self.params[self.stage] -= self.delta[self.stage]
            self.delta[self.stage] *= -0.8

        self.params[self.stage] = max(self.params[self.stage] + self.delta[self.stage], 0)
        
        # if abs(self.delta[self.stage]) < 0.001:
        #     self.stage += 1
        #     if self.stage >= 3:
        #         self.tuning_mode = False
        #         self.params = copy.deepcopy(self.best_params)
        #     self.delta[self.stage] = 0.5 if self.stage == 0 else 0.01

        # self.accumulated_error = 0.0

        if abs(self.delta[self.stage]) < 0.001:
            if not self.stage_completed:
                self.stage += 1
                if self.stage >= 3:
                    self.tuning_mode = False
                    self.params = self.best_params.copy()
                else:
                    # Сброс для следующего этапа
                    self.delta[self.stage] = 0.5 if self.stage == 0 else 0.01
                    self.best_error = float('inf')
                self.stage_completed = True
        else:
            self.stage_completed = False

        self.accumulated_error = 0.0
    


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
        self.max_abs_z_speed = 3.0
        self.base_w_speed = 1.5

        self.pid_side = PID(25, 0, 0, -self.max_abs_z_speed, self.max_abs_z_speed, self.dt)
        self.pid_front = PID(20, 0, 0, 0, self.max_abs_z_speed, self.dt)
        
        self.autotuner = AutoTuner([25.0, 0.0, 0.0], self.dt)
        self.total_time = 0.0
        self.max_tuning_time = 42.0  # Максимальное время настройки (30 сек)

        
    def update_mode(self, msg):
        with self.mode_lock:
        #     if msg.data == 1 and self.mode == 0:
        #         self.autotuner = AutoTuner([25.0, 0.0, 0.0], self.dt)
        #         self.total_time = 0.0  # Сброс времени
        #     self.mode = msg.data
            self.mode = msg.data


    def update_distances(self, msg):
        data = json.loads(msg.data)
        data = {int(k): v for k, v in data.items()}

        #
        if any(value == -1 for value in self.lidar_basic.values()): ## TODO КОСТЫЛЬ ДЛЯ ДЕБАГА!!! УБРАТЬ!!!
            if self.mode == 0:
                self.mode = 1
        #
        with self.lidar_lock:
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
            lin_x = 0.0 

            side_error = self.target_border_dist - lidar_data[90]
            front_error = self.front_turn_dist - lidar_data[0] if lidar_data[0] != -1 else 0

            # Обновление с использованием времени
            if self.autotuner.tuning_mode and self.total_time < self.max_tuning_time:
                self.autotuner.update_error(side_error)  # Убираем параметр расстояния
                self.pid_side.p = self.autotuner.params[0]
                self.pid_side.i = self.autotuner.params[1]
                self.pid_side.d = self.autotuner.params[2]
            else:
                with ('pid.txt', 'a') as file:
                    file.write('\n')
                    file.write(f'p={self.pid_side.p}, i={self.pid_side.i}, d={self.pid_side.d}')

            ang_w = self.pid_side.calculate(side_error)
            self.get_logger().info(f'side err {side_error}, {ang_w}')
            lin_x = self.base_x_speed

            if lidar_data[0] < self.front_turn_dist:
                #ang_w = self.pid_front.calculate(front_error)
                ang_w = self.base_w_speed
                self.get_logger().info(f'font err {front_error}, {ang_w}')

            msg.linear.x = lin_x
            msg.angular.z = ang_w

            self.total_time += self.dt

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