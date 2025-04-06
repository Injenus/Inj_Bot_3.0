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
        self.subscription = self.create_subscription(Image, 'cam/binocular_g', self.image_callback, 1)
        self.publisher = self.create_publisher(String, 'qr_codes', 1)

    def order_points(self, pts):
        """Упорядочивает точки в порядке: RT → LT → LB → RB."""
        if len(pts) != 4:
            return pts

        # Конвертируем в numpy массив
        pts_np = np.array(pts, dtype="float32")
        
        # 1. Разделяем точки на верхние и нижние по Y-координате
        y_sorted = pts_np[np.argsort(pts_np[:, 1])]
        top = y_sorted[:2]  # Две верхние точки
        bottom = y_sorted[2:]  # Две нижние точки

        # 2. Сортируем верхние точки по X (от большего к меньшему: RT → LT)
        top_sorted = top[np.argsort(top[:, 0])[::-1]]
        
        # 3. Сортируем нижние точки по X (от меньшего к большему: LB → RB)
        bottom_sorted = bottom[np.argsort(bottom[:, 0])]

        # Собираем итоговый порядок
        ordered = np.vstack([top_sorted, bottom_sorted])
        return [tuple(ordered[i].astype(int)) for i in range(4)]

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Ошибка конвертации: {str(e)}')
            return

        decoded_objects = decode(cv_image)
        qr_dict = {}

        for idx, obj in enumerate(decoded_objects):
            if obj.type != 'QRCODE':
                continue

            data = obj.data.decode('utf-8')
            points = [(int(p.x), int(p.y)) for p in obj.polygon]  # Явное преобразование в int
            ordered_points = self.order_points(points)
            
            # Преобразование numpy-значений в стандартные типы Python
            x_center = int(np.mean([p[0] for p in ordered_points]))/cv_image.shape[1]
            y_center = int(np.mean([p[1] for p in ordered_points]))/cv_image.shape[0]

            # Сохраняем данные как вложенный словарь с примитивными типами
            qr_dict[str(idx)] = (
                (x_center, y_center),
                ordered_points,
                data
            )

        if qr_dict:
            # Кастомный сериализатор
            def custom_serializer(obj):
                if isinstance(obj, (np.integer, np.floating)):
                    return int(obj) if isinstance(obj, np.integer) else float(obj)
                if isinstance(obj, (tuple, list)):
                    return [custom_serializer(item) for item in obj]
                return obj

            try:
                json_str = json.dumps(qr_dict, default=custom_serializer)
                msg = String()
                msg.data = json_str
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Ошибка сериализации: {str(e)}')

        self.get_logger().info(f'Обнаружено QR-кодов: {len(qr_dict)}')
        self.get_logger().info(f'{qr_dict}')


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()