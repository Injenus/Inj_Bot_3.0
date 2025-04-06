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

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from _tools import *


class ArucoGrayscaleDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        self.declare_parameter('dictionary_size', 5)
        dictionary_size = self.get_parameter('dictionary_size').value
        
        self.aruco_dict = aruco.Dictionary_get(
            aruco.DICT_4X4_50 if dictionary_size == 4 else 
            aruco.DICT_5X5_50 if dictionary_size == 5 else
            aruco.DICT_6X6_50 if dictionary_size == 6 else
            aruco.DICT_7X7_50
        )
        
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(Image, 'cam/binocular_g', self.image_callback, 2)
        self.publisher = self.create_publisher(String, 'aruco_markers', 2)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        corners, ids, _ = aruco.detectMarkers(
            cv_image, 
            self.aruco_dict,
            parameters=self.parameters
        )

        result = {}
        
        if ids is not None:
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]
                
                ordered_corners, orientation = self.process_corners(marker_corners)
                center = np.mean(marker_corners, axis=0)
                
                result[int(marker_id)] = (
                    (float(center[0]/cv_image.shape[1]), float(center[1]/cv_image.shape[0])),
                    tuple(tuple(map(float, p)) for p in ordered_corners),
                    int(orientation) 
                )
                # cv2.imwrite(f'/home/inj/Inj_Bot_3.0/test_{i}_{round(center[0]/cv_image.shape[1], 3)}_{round(center[1]/cv_image.shape[0], 3)}.jpg', cv_image)

        output = String()
        output.data = json.dumps(result)
        self.publisher.publish(output)
        self.get_logger().info(f'Отправили: {output.data}')

    def process_corners(self, corners):
        """Определение порядка углов и ориентации по исходному верхнему левому углу"""
        # Находим исходный верхний левый угол
        sums = np.sum(corners, axis=1)
        top_left_idx = np.argmin(sums)
        
        # Определяем ориентацию и порядок углов
        order_mapping = {
            0: ([1, 0, 3, 2], 0),    # нормальное положение
            1: ([2, 1, 0, 3], 270),  # поворот на 270°
            2: ([3, 2, 1, 0], 180),  # поворот на 180°
            3: ([0, 3, 2, 1], 90)    # поворот на 90°
        }
        
        ordered_indices, orientation = order_mapping[top_left_idx]
        ordered_corners = corners[ordered_indices].tolist()
        
        return ordered_corners, orientation

def main(args=None):
    rclpy.init(args=args)
    node = ArucoGrayscaleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()