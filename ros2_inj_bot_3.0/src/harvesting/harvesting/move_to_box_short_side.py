"""
идея - заезжать всегда в среднюю ячеку (2 и 5), чтобы кидать по приерно центру ящика по кортоокму ребру
нельзя просто доехать как за овощем - далеко, поэтому
    доезжаем в среднюю ячейку- - по растсонию переда (2) / зада(5)
    враще5мся по часовой стрелке пока не будем задом на нужно растсонии (от бокса)
    едем назад до "вполтную"
    вращаемся противо часово пока левый борт на будет на "растсонии"
    попрото манипулятора, кидаем
    вращемся по часовой пока перед не будет на растсоние от бюрдюрап
    едем впередб до растионию до бордюра
    вращзаемс против чаосвой пока левый борт не будет на расчсятонии от бюордюра

    "продолжаем миссию"

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from threading import Lock
import json
import copy
import math

import sys, os
modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf

class MoveToBox():
    def __int__(self):
        super().__init__('move_to_box_short')

        self.subs_block_number = self.create_subscription(UInt8, 'block_num', self.update_block_num, 10)
        self.subs_mode = self.create_subscription(UInt8, 'mode', self.update_mode, 10)
        self.subs_lidar = self.create_subscription(String, 'lidar/obstacles', self.update_distances, 3)

        self.publ_twist = self.create_publisher(Twist, '/cmd_vel', 3)
        self.publ_arm = self.create_publisher(String, 'arm_action', 10)
        self.publ_status = self.create_publisher(UInt8, 'short_throw_status', 10)

        self.block_number = 0
        self.mode = 0
        
        self.dist_toler = conf.distance_tolerance
        self.basic_x = conf.base_linear_x_speed
        self.basic_w = conf.base_angular_w_speed

        self.mode_locker = Lock()
        self.block_locker = Lock()
        self.lidar_locker = Lock()

        self.block_state = 0
        self.box_state = 0

        dt = conf.dt
        self.timer = self.create_timer(dt, self.loop)
        self.waiter = [0, int(round(conf.wait_seconds / dt))]
    

    def update_block_num(self, msg):
        with self.block_locker:
            if msg.data != self.block_number:
                self.block_number = msg.data

    def update_mode(self, msg):
        with self.mode_locker:
            if msg.data != self.mode:
                self.mode = msg.data


    def update_distances(self, msg):
        data = json.loads(msg.data)
        data = {int(k): v for k, v in data.items()}

        with self.lidar_locker:
            self.lidar_basic = {
                0: data[0],
                90: data[90],
                180: data[180],
                270: data[270]
            }

        with self.mode_locker:
            mode = self.mode
            if any(not isinstance(value, float) or not isinstance(value, int) for value in self.lidar_basic.values()):
                self.mode = -2
                self.get_logger().info(f"NOT NUMERIC LIDAR DATA")
                #raise ValueError(f'NOT NUMERIC LIDAR DATA {self.lidar_basic}')
            else:
                self.mode = mode
            if any(value < 0 for value in self.lidar_basic.values()):
                self.mode = -3
                #raise ValueError(f'NEGATIVE LIDAR DATA {self.lidar_basic}')
                self.get_logger().info(f"NEGATIVE LIDAR DATA")
            else:
                self.mode = mode

        
    def loop(self):
        lin_x, ang_w = 0.0, 0.0
        with self.mode_locker:
            mode = self.mode
        with self.block_locker:
            block_num = self.block_number
        with self.lidar_locker:
            lidar_data = self.lidar_basic

        if mode == 1:

            if self.block_state == 0:

                if block_num == 1:
                    if lidar_data[0] > conf.front_dist_to_get_middle_cell:
                        lin_x = self.basic_x
                    else:
                        self.block_state = 1

                elif block_num == 2:
                    self.block_state = 1

                elif block_num == 3:
                    if lidar_data[0] < conf.front_dist_to_get_middle_cell:
                        lin_x = -self.basic_x
                    else:
                        self.block_state = 1

                elif block_num == 4:
                    if lidar_data[180] < conf.back_dist_to_get_middle_cell:
                        lin_x = self.basic_x
                    else:
                        self.block_state = 2

                elif block_num == 5:
                    self.block_state = 2

                elif block_num == 6:
                    if lidar_data[180] > conf.back_dist_to_get_middle_cell:
                        lin_x = -self.basic_x
                    else:
                        self.block_state = 2


            elif self.block_state in [1, 2]:

                if self.box_state == 0: # находимся в средней ячекйи, слева ящик
                    if lidar_data[180] > conf.back_dist_to_turn_to_box:
                        ang_w = -self.basic_w # вращение по часовой
                    else:
                        self.box_state = 1

                elif self.box_state == 1: # находмся спиной к коробке
                    if lidar_data[180] > conf.back_dist_to_box:
                        lin_x = -self.basic_x
                    else:
                        self.box_state = 2

                elif self.box_state == 3:# находимся задом к коробке вполнтую
                    if lidar_data[270] > conf.left_lidar_dist_to_turn_to_box:
                        ang_w = self.basic_w
                    else:
                        self.box_state = 4

                elif self.box_state == 4:
                    msg = String()
                    msg.data = 'throw_short_side'
                    self.publ_arm.publish(msg)
                    self.box_state = 5
                
                elif self.box_state == 5:
                    if self.waiter[0] < self.waiter[1]:
                        self.waiter[0] += 1
                    else:
                        self.waiter[0] = 0
                        self.box_state = 6
                
                elif self.box_state == 6:
                    if lidar_data[0] > conf.front_dist_to_go_away_from_box:
                        ang_w = -self.basic_w
                    else:
                        self.box_state = 7

                elif self.box_state == 7:
                    if lidar_data[0] > conf.front_dist_to_go_to_border:
                        lin_x = self.basic_x
                    else:
                        self.box_state = 8
                
                elif self.box_state == 8:
                    # if lidar_data[90] > 0.28:
                    #     ang_w = self.basic_w
                    # else:
                    if 1:
                        self.box_state = 9
                        msg = UInt8
                        msg.data = 1
                        self.publ_status.publish(msg)

                        with self.mode_locker:
                            self.mode = 0
        

        elif mode == 0:
            pass
        elif mode in [-2, -3]:
            pass # обычно, если лидар упал нахуй. если нет - то мб прошло бы от мелких дрожаний, но похуй

        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.w = ang_w
        self.publ_twist.publish(msg)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = MoveToBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()                  



                    
                    





        



