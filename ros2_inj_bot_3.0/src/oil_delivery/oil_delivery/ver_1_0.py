import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, UInt8MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import numpy as np
from std_msgs.msg import Int32MultiArray

receive_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if receive_data_path not in sys.path:
    sys.path.append(receive_data_path)

import play_audio

current_script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(current_script_path)
log_patterns_path = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..', '..', 'src', 'log_patterns'))

class Oil_Delivery(Node):
    def __init__(self):
        super().__init__('oil_delivery')
        self.bridge = CvBridge()

        self.servo_pub = self.create_publisher(UInt8MultiArray, 'servo/to_write', 1)
        self.subscription = self.create_subscription(Image, 'cam/arm', self.search_qr, 1)
        self.wheel_sub = self.create_subscription(Int32MultiArray, 'wheel/target_rpm', self.wheel_data, 3)
        
        self.waiting_for_qr = False
        self.window_shown = False
        self.first_data_received = False  # Новый флаг для первого изображения
        self.initial_command_timer = None  # Таймер для начальной команды

    def initial_servo_command(self):
        # Отправляем первую команду и отменяем таймер
        self.publish_servo_command(bytes([142-90, 133+75, 130-120, 132, 0, 128, 42]))
        if self.initial_command_timer:
            self.initial_command_timer.cancel()
        self.waiting_for_qr = True

    def publish_servo_command(self, data):
        servo_msg = UInt8MultiArray()
        servo_msg.data = data
        self.servo_pub.publish(servo_msg)

    def wheel_data(self, msg):
        # Запускаем таймер при первом получении изображения
        if not self.first_data_received:
            self.first_data_received = True
            self.get_logger().info("Первые уставки получены")
            self.initial_command_timer = self.create_timer(2.0, self.initial_servo_command)

    def search_qr(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(frame)

        if data:
            self.get_logger().info(f"Найден QR-код: {data}")
            play_audio.main()
            if self.waiting_for_qr:
                self.publish_servo_command(bytes([142, 133, 130, 132, 0, 128, 42]))
                self.waiting_for_qr = False

            if not self.window_shown:
                self.show_qr_window(data)

    def show_qr_window(self, qr_data):
        # Параметры экрана (можете изменить под ваше разрешение)
        screen_width = 640
        screen_height = 480

        # Создаем желтое изображение
        yellow_img = np.zeros((screen_height, screen_width, 3), dtype=np.uint8)
        yellow_img[:] = (0, 255, 255)  # BGR для желтого цвета

        # Настраиваем текст
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 2
        thickness = 3
        text_color = (0, 0, 0)  # Черный текст

        # Вычисляем позицию текста
        text_size = cv2.getTextSize(qr_data, font, font_scale, thickness)[0]
        text_x = (screen_width - text_size[0]) // 2
        text_y = (screen_height + text_size[1]) // 2

        # Добавляем текст на изображение
        cv2.putText(yellow_img, qr_data, (text_x, text_y), 
                    font, font_scale, text_color, thickness, cv2.LINE_AA)

        # Создаем полноэкранное окно
        cv2.namedWindow("QR Code", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("QR Code", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("QR Code", yellow_img)
        cv2.waitKey(1)  # Обновляем окно

        # Устанавливаем флаг и запускаем таймер для закрытия окна
        self.window_shown = True
        self.window_timer = self.create_timer(5.0, self.close_qr_window)
        
    def close_qr_window(self):
        cv2.destroyWindow("QR Code")
        self.window_shown = False
        self.window_timer.cancel()
        
    def destroy_node(self):
        if self.initial_command_timer:
            self.initial_command_timer.cancel()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Oil_Delivery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()