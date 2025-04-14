"""
в общем случае сущетсвует 12 маршрутов - по 2 маршрута от каждого из 6 блоков.
один маршрут приезжает для броска по короткому ребру, другой - по длинному

нумерауия блоков от парвого нижнего против чаосвой стрелки

начальное положение движения из блока неизветсно

гипотетичски, ПО КОРОТКОМУ РЕБРУ находясь в блоках 2 и 5 можно бросить по короткому ребру без движения,
соответвенно для брсока из блолка 1 - подъзежаем в бллок 2, 
из 3 - возвращемся в 2
из 4 - подлъзжаем в 5
из 6 - возвращемся в 5
короткий реалдизован в координаторе + реверс движения бордюра

для брсока ПО ДЛИННОМУ РЕБРУ (давайте может нахуй, короткого хватит?)

из блока 6:
    вращаемся против чаосво пока переднее растояние не станет равно <>
    едем вперед пока переднее расстоние не станет равно  <>
    кидаем
    едем назад пока заднее расстоние не станет равно <>
    вращаемся по часовой стрелке по правый луч не стент равен <>
    (едем вперёд пока заднее расстоние не станет равно <> / едем <> секунд)
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

class MoveToBox():
    def __int__(self):
        super().__init__('move_to_box')

        self.subs_block_number = self.create_subscription(UInt8, 'block_num', self.update_block_num, 10)
        self.subs_mode = self.create_subscription(UInt8, 'mode', self.update_mode, 10)
        self.subs_lidar = self.create_subscription(String, 'lidar/obstacles', self.update_distances, 3)

        self.publ_twist = self.create_publisher(Twist, '/cmd_vel', 3)
        self.publ_arm = self.create_publisher(String, 'arm_action', 10)

        self.block_number = 0
        self.mode = 0
        self.state = 0
        
        self.dist_toler = 0.02

        self.mode_locker = Lock()
        self.block_locker = Lock()
        self.lidar_locker = Lock()

        dt = 0.005
        self.timer = self.create_timer(dt, self.loop)
        self.waiter = [0, int(round(3.0 / dt))]
    


    def update_block_num(self, msg):
        with self.block_locker:
            self.block_number = msg.data

    def update_mode(self, msg):
        with self.mode_locker:
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

        if any(not isinstance(value, float) or not isinstance(value, int) for value in self.lidar_basic.values()):
            raise ValueError(f'NOT NUMERIC LIDAR DATA {self.lidar_basic}')
        if any(value < 0 for value in self.lidar_basic.values()):
            raise ValueError(f'NEGATIVE LIDAR DATA {self.lidar_basic}')
        
    def loop(self):
        with self.mode_locker:
            mode = self.mode
        with self.block_locker:
            block_num = self.block_number
        with self.lidar_locker:
            lidar_data = self.lidar_basic
        
        if mode == 1:
            
            if block_num == 6:
                    lin_x = 0.0
                    ang_w = 0.0

                    if self.state == 0:
                        if abs(lidar_data[0] - 0.9) > self.dist_toler:
                            ang_w = - 1.5
                        else:
                            self.state = 1
                            ang_w = 0.0

                    elif self.state == 1:
                        if abs(lidar_data[0] - 0.2) > self.dist_toler:
                            lin_x  = 0.12
                        else:
                            self.state = 2
                            lin_x = 0.0

                    elif self.state == 2:
                        if self.waiter[0] == 0:
                            msg = String()
                            msg.data = "throw_long_side"
                            self.publ_arm.publish(msg)
                            self.waiter[0] += 1
                        elif self.waiter[0] < self.waiter[1]:
                            self.waiter[0] += 1
                        else:
                            self.waiter[0] = 0
                            self.state = 3

                    elif self.state == 3:
                        if abs(lidar_data[0] - 0.9) > self.dist_toler:
                            lin_x = -0.12
                        else:
                            self.state = 4
                            lin_x = 0.0

                    elif self.state == 4:
                        if abs(lidar_data[90 - 0.28]) > self.dist_toler:
                            ang_w = 1.5
                        else:
                            self.state = 5
                            ang_w = 0.0

                    elif self.state == 5:
                        if abs(lidar_data[180] - 2.9) > self.dist_toler:
                            lin_x = 0.16
                        else:
                            self.state = 6
                            lin_x = 0.0

                    elif self.state == 6:
                        if self.waiter[0] < self.waiter[1] * 10:
                            self.waiter[0] += 1
                        else:
                            self.state = 0 # ОЧЕНЬ ОПАСНЫЙ МОМЕНТ но лень ебатсья с поткоами ради не нужного метода
                            self.waiter[0] = 0

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



                    
                    





        



