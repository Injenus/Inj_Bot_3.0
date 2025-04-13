import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import copy
import numpy as np
import time
import bisect

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from play_audio import play_audio

"""
 0 включаем песню
 1 - вращаемся по час
 2 - пауза
 3 - вращемся прот час
"""


class Uiiaiuiiai(Node):
    def __init__(self):
        super().__init__('uiiaiuiiai')

        self.main_state = 0
        self.first_speed = 5.0
        self.second_speed = -20.0

        self.publ_twist = self.create_publisher(Twist, '/cmd_vel', 3)

        self.timer = self.create_timer(0.005, self.loop)


        self.pause = self.create_timer(2.4, self.wait)
        self.pause.reset() # отмена пертиодического вызова

        self.resume = self.create_timer(3.6, self.change_direction)
        self.resume.reset()

        self.node_stop = self.create_timer(5.8, self.destroy_node)
        self.node_stop.reset()


    def wait(self):
        self.main_state = 2
        self.pause.cancel()

    def change_direction(self):
        self.main_state = 3
        self.resume.cancel() 


    def loop(self):
        msg = Twist()

        if self.main_state == 0:
            play_audio('uiiaiuiiai.wav')
            self.main_state = 1

        if self.main_state == 1:
            msg.angular.z = self.first_speed

        if self.main_state == 2:
            msg.angular.z = 0.0

        if self.main_state == 3:
            msg.angular.z = self.second_speed

        self.publ_twist.publish(msg)

    
    def destroy_node(self):
        self.main_state = 4
        msg = Twist()
        self.publ_twist.publish(msg)
        self.node_stop.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    uiiaiuiiai = Uiiaiuiiai()
    rclpy.spin(uiiaiuiiai)

if __name__ == '__main__':
    main()


