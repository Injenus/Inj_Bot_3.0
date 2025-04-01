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

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        
        self.subscription = self.create_subscription(Image, 'cam/arm_g', self.image_callback, 1)
        self.publisher = self.create_publisher(String, 'qr_codes', 1)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Ошибка конвертации: {str(e)}')
            return

        # Детекция и декодирование QR-кодов
        ret, decoded_info, points, _ = self.detector.detectAndDecodeMulti(cv_image)
        
        qr_dict = {}
        if ret:
            for i, (data, qr_points) in enumerate(zip(decoded_info, points)):
                # Преобразование координат углов
                qr_points = qr_points.reshape(4, 2).astype(int)
                
                # Переупорядочивание углов
                ordered_points = [
                    tuple(qr_points[0]),  # Верхний-Левый (Top-Left)
                    tuple(qr_points[1]),  # Верхний-Правый (Top-Right)
                    tuple(qr_points[2]),  # Нижний-Правый (Bottom-Right)
                    tuple(qr_points[3])   # Нижний-Левый (Bottom-Left)
                ]
                
                x_center = int(np.mean(qr_points[:, 0]))
                y_center = int(np.mean(qr_points[:, 1]))

                qr_dict[i] = (
                    (x_center, y_center),
                    tuple(ordered_points),
                    data
                )
            json_str = json.dumps(qr_dict, default=lambda o: list(o) if isinstance(o, tuple) else o)

            msg = String()
            msg.data = json.dumps(json_str, ensure_ascii=False)
            self.publisher.publish(msg)

        self.get_logger().info(f'Обнаружено QR-кодов: {len(qr_dict)}')
        self.print_qr_info(qr_dict)

    def print_qr_info(self, qr_dict):
        for idx, info in qr_dict.items():
            self.get_logger().info(
                f'QR {idx}: Центр: {info[0]}, '
                f'Углы: {info[1]}, '
                f'Данные: "{info[2]}"'
            )

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()