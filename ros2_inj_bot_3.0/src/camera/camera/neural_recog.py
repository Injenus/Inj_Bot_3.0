import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
from datetime import datetime
import sys
from std_msgs.msg import String
import cv2.aruco as aruco
import json
import numpy as np


class NeuralRecognizer(Node):
    def __init__(self):
        super().__init__('neural_recognizer')



        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(Image, 'cam/arm', self.image_callback, 2)
        self.publisher = self.create_publisher(String, 'aruco_markers', 2)