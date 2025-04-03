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

venv_site_packages = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'venv', 'lib', 'python3.11', 'site-packages')
if venv_site_packages not in sys.path:
    sys.path.append(venv_site_packages)
from pyzbar.pyzbar import decode

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)
from _tools import *

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 'cam/arm_g', self.image_callback, 1)
        self.publisher = self.create_publisher(String, 'qr_codes', 1)

    def order_points(self, pts):
        """Упорядочивает точки в порядке: верхний-левый, верхний-правый, нижний-правый, нижний-левый."""
        if len(pts) != 4:
            return pts
        pts_np = np.array(pts, dtype="float32")
        rect = np.zeros((4, 2), dtype="float32")
        s = pts_np.sum(axis=1)
        rect[0] = pts_np[np.argmin(s)]
        rect[2] = pts_np[np.argmax(s)]
        diff = np.diff(pts_np, axis=1)
        rect[1] = pts_np[np.argmin(diff)]
        rect[3] = pts_np[np.argmax(diff)]
        return [tuple(rect[i].astype(int)) for i in range(4)]

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Ошибка конвертации: {str(e)}')
            return

        decoded_objects = decode(cv_image)
        qr_dict = {}

        for idx, obj in enumerate(decoded_objects):
            if obj.type != 'QRCODE':
                continue

            data = obj.data.decode('utf-8')
            points = [(p.x, p.y) for p in obj.polygon]
            ordered_points = self.order_points(points)
            
            x_coords = [p[0] for p in ordered_points]
            y_coords = [p[1] for p in ordered_points]
            x_center = int(np.mean(x_coords))
            y_center = int(np.mean(y_coords))

            qr_dict[idx] = (
                (x_center, y_center),
                ordered_points,
                data
            )

        if qr_dict:
            json_str = json.dumps(qr_dict, default=lambda o: list(o) if isinstance(o, tuple) else o)
            msg = String()
            msg.data = json_str
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