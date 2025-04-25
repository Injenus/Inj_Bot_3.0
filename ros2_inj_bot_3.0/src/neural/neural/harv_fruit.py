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
from ultralytics import YOLO
import sys, os

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from _tools import *

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'neural', 'neural')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import neural_matching as nm


class NeuralHarvFruit(Node):
    def __init__(self):
        super().__init__('neural_harv_fruit')

        self.declare_parameter('neural_id', 0)
        self.declare_parameter('cam_topic', 'cam/arm')
        self.neural_id = self.get_parameter('neural_id').value
        self.cam_topic = self.get_parameter('cam_topic').value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, self.cam_topic, self.image_callback, 2)
        self.publisher = self.create_publisher(String, 'img_classif', 3)

        self.ncnn_model = YOLO(nm.neural_data[self.neural_id][0])
        self.class_names = nm.neural_data[self.neural_id][1]

    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.ncnn_model(frame, imgsz=224, conf_thres=0.75, iou_thres=0.6)[0]

        last_class, last_x = '', -1
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            class_name = self.class_names.get(cls_id, 'unknown')
            last_class = class_name
            last_x = ((x1+x2)/2)/frame.shape[1]

            #print(f"Class ID: {cls_id}, Name: {class_name}, Confidence: {conf:.2f}")

            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        cv2.imshow(f'{self.cam_topic}_{self.neural_id}_Harvesting', resize(0.75, frame))
        cv2.waitKey(1)
        print(f'objs num  {len(results)}')

        if len(results) == 1:
            if last_class != "unknown":
                msg = String()
                img_data = {'class': last_class, 'x': last_x}
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
    nuer_recog = NeuralHarvFruit()
    rclpy.spin(nuer_recog)
    nuer_recog.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        