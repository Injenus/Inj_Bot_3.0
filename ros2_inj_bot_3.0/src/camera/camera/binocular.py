import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from _tools import *

class BinocularCameraPublisher(Node):
    def __init__(self):
        super().__init__('binocular_camera_publisher')
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'cam/binocular', 1)
        self.publ_gray = self.create_publisher(Image, "cam/binocular_g", 1)

        generated_value = self._get_cam_index()
        self.declare_parameter('cam_idx', generated_value)
        cam_index = self.get_parameter('cam_idx').value
        self.get_logger().info(f"Определён индекс: {cam_index}")

        self.cam = cv2.VideoCapture(cam_index) # 2560 X 720
        self.cam.set(cv2.CAP_PROP_FPS, 30)

        if not self.cam.isOpened():
            self.get_logger().info(f"Не удалось открыть камеру {cam_index} :(")
            self.destroy_node()
            rclpy.try_shutdown() 
            sys.exit(0) 
        
        self.timer = self.create_timer(0.03, self.publish_imge)

    def publish_imge(self):
        ret, frame = self.cam.read()

        if ret:
            frame = cv2.rotate(frame,cv2.ROTATE_180)
            #frame = resize(4, frame)
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(image_msg)
            self.get_logger().info('Published frame from Binocular_Cam')
            
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_msg = self.bridge.cv2_to_imgmsg(gray_frame, 'mono8')
            self.publ_gray.publish(gray_msg)

    def _get_cam_index(self):

        def try_open(id):
            cap = cv2.VideoCapture(i)
            if not cap.isOpened():
                print(f"Не удалось открыть камеру с индексом {i}")
            else:
                ret, frame = cap.read()
                if ret:
                    return i
                else:
                    None

        i = 0
        while i < 42:
            idx = try_open(i)
            if idx is not None:
                return idx
            else:
                i += 1


    
def main(args=None):
    rclpy.init(args=args)
    binocular_cam = BinocularCameraPublisher()
    rclpy.spin(binocular_cam)
    binocular_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()