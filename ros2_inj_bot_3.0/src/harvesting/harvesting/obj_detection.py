import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
from threading import Thread, Event, Lock
import json
import copy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from _tools import *

import sys, os
modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf


class FruitDetector(Node):
    def __init__(self):
        super().__init__('fruit_detector')

        self.arm_subs = self.create_subscription(Image, 'cam/arm', self.get_image, 2)
        self.publisher = self.create_publisher(String, 'img_classif', 10)



    
    def get_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')



        cv2.imshow('Arm', resize(0.5, frame))
        cv2.waitKey(1)



        msg = String()
        self.publisher.publish(msg)


    def destroy_node(self):
        if self.out:
            self.out.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    fruit_detector = FruitDetector()
    rclpy.spin(fruit_detector)
    fruit_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()