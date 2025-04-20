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
        


    def get_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        frame = crop_image(frame, 150, 150, 0, 0)

        sample_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        sample_image_tensor = transform(sample_image).unsqueeze(0) # Добавляем размерность батча
        with torch.no_grad():
            output = self.classif_model(sample_image_tensor)
            _, predicted_class = torch.max(output, 1)
            predicted_label = predicted_class.item()

        fruit_class = conf.id_to_class[predicted_label] # string
        x_pos = 0.51

        cv2.putText(frame, fruit_class, (frame.shape[1]//2, frame.shape[0]//2), cv2.FONT_HERSHEY_SIMPLEX, 
                   5, (0,255,0), 3)


        # cv2.imshow('Arm', frame)
        # cv2.waitKey(1)

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