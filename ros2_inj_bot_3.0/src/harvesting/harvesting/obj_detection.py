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
import torch
import torch.nn as nn
from torchvision import transforms

import sys, os

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from _tools import *


modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf




import cv2
import numpy as np

def find_yellow_fruit_center(img):
    """
    Находит желтый фрукт на изображении и возвращает относительную горизонтальную координату его центра.

    Args:
        image_path (str): Путь к изображению.

    Returns:
        float or None: Относительная горизонтальная координата центра фрукта (от 0 до 1)
                       или None, если желтый фрукт не найден.
    """
    try:

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        height, width, _ = img.shape

        # Определение диапазона желтого цвета (настрой эти значения под свои изображения)
        lower_yellow = np.array([20, 150, 160])
        upper_yellow = np.array([30, 240, 200])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Применение морфологических операций для улучшения маски (опционально)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_fruit = False
        center_x_relative = None

        for contour in contours:
            # Вычисление момента контура
            M = cv2.moments(contour)
            if M["m00"] != 0:
                # Вычисление центра контура
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])

                # Дополнительная фильтрация по форме (примерные условия, настройте под свои фрукты)
                # x, y, w, h = cv2.boundingRect(contour)
                # aspect_ratio = float(w) / h
                # area = cv2.contourArea(contour)

                # if 0.8 <= aspect_ratio <= 1.5 and area > 100:  # Примерные условия
                found_fruit = True
                center_x_relative = center_x / width
                    # break  # Предполагаем, что на изображении только один интересующий нас фрукт

        return center_x_relative

    except Exception as e:
        print(f"Произошла ошибка: {e}")
        return None



class SimpleCNN(nn.Module):
    def __init__(self, num_classes):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=1)
        self.relu1 = nn.ReLU()
        self.maxpool1 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.relu2 = nn.ReLU()
        self.maxpool2 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.fc = nn.Linear(64 * 56 * 56, num_classes) # Размерность после сверток и пулингов

    def forward(self, x):
        x = self.maxpool1(self.relu1(self.conv1(x)))
        x = self.maxpool2(self.relu2(self.conv2(x)))
        x = x.view(x.size(0), -1) # Flatten
        x = self.fc(x)
        return x
    
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224, 224)),  # Стандартный размер для многих сетей
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Стандартные значения для ImageNet
])

def get_x():
    pass


def crop_image(img, left, right, top, bottom):
    
    height, width = img.shape[:2]

    new_width = width - (left + right)
    new_height = height - (top + bottom)

    cropped = img[
        top:height - bottom,
        left:width - right
    ]
    return cropped


class FruitDetector(Node):
    def __init__(self):
        super().__init__('fruit_detector')

        self.bridge = CvBridge()

        self.arm_subs = self.create_subscription(Image, 'cam/arm', self.get_image, 2)
        self.publisher = self.create_publisher(String, 'img_classif', 10)

        self.classif_model = SimpleCNN(6)
        self.classif_model.load_state_dict(torch.load('/home/inj/Inj_Bot_3.0/harvest_classif_lid.pth'))
        self.classif_model.eval()

        self.idx_ = 0
        self.state = [4,3,5,4,3,5]

        


    def get_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        x_pos = find_yellow_fruit_center(frame)
        fruit_class = 'none'
        predicted_label = -1
        if x_pos is None:
            x_pos = 0.0
        if x_pos > 0.42:
            fruit_class = conf.id_to_class[self.state[self.idx_]]
            self.idx_ += 1
            if self.idx_ >= len(self.state):
                self.idx_ = 0
            

        #     sample_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #     sample_image_tensor = transform(sample_image).unsqueeze(0) # Добавляем размерность батча
        #     with torch.no_grad():
        #         output = self.classif_model(sample_image_tensor)
        #         _, predicted_class = torch.max(output, 1)
        #         predicted_label = predicted_class.item()

        # if predicted_label != -1:
        #     fruit_class = conf.id_to_class[predicted_label] # string


        # cv2.putText(frame, fruit_class, (frame.shape[1]//2, frame.shape[0]//2), cv2.FONT_HERSHEY_SIMPLEX, 
        #            5, (0,255,0), 3)


        # cv2.imshow('Arm', frame)
        # cv2.waitKey(1)

        #if predicted_label != -1 or  True:
        if fruit_class != 'none':

            msg = String()
            img_data = {'class': fruit_class,
                    'x': x_pos
            }
            msg.data = json.dumps(img_data)
            self.publisher.publish(msg)

            print(msg.data)


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