import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

current_script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(current_script_path)
print(script_dir)
receive_data_path = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..', '..', '..', 'modules'))
print(receive_data_path)
if receive_data_path not in sys.path:
    sys.path.append(receive_data_path)

from _tools import *

class BinocularCameraPublisher(Node):
    def __init__(self):
        super().__init__('binocular_camera_publisher')
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'cam/binocular', 1)
        cam_num = 8
        self.cam = cv2.VideoCapture(cam_num)

        if not self.cam.isOpened():
            self.get_logger().info(f"Не удалось открыть камеру {cam_num} :(")
            self.destroy_node()
            rclpy.try_shutdown() 
            sys.exit(0) 
        
        self.timer = self.create_timer(0.05, self.publish_imge)

    def publish_imge(self):
        ret, frame = self.cam.read()

        if ret:
            #rame = resize(2, frame)
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(image_msg)
            self.get_logger().info('Published frame from Binocular_Cam')

    
def main(args=None):
    rclpy.init(args=args)
    binocular_cam = BinocularCameraPublisher()
    try:
        rclpy.spin(binocular_cam)
    except:
        binocular_cam.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()