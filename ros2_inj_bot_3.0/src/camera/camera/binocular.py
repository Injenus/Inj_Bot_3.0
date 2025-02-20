import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BinocularCameraPublisher(Node):
    def __init__(self):
        super.__init__('binocular_camera_publisher')
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'cam/binocular', 1)

        self.cam = cv2.VideoCa