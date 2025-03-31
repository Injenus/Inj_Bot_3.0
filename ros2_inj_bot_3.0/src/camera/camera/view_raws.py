import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from _tools import *

class CamerasSubscriber(Node):
    def __init__(self):
        super().__init__('cameras_subscriber')
        self.bridge = CvBridge()

        self.subscription_arm = self.create_subscription(
            Image, 'cam/arm', self.arm_display, 1
        )
        
        self.subscription_binocular = self.create_subscription(
            Image, 'cam/binocular', self.binocular_display, 1
        )

        self.save_dir = os.path.join('/home', 'inj', 'Inj_Bot_3.0')
        os.makedirs(self.save_dir, exist_ok=True)

        self.arm_last_time = None
        self.arm_fps = 0.0
        self.binocular_last_time = None
        self.binocular_fps = 0.0

    
    def arm_display(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        current_time = self.get_clock().now()
        
        # Расчет FPS
        if self.arm_last_time is not None:
            delta_time = current_time - self.arm_last_time
            delta_sec = delta_time.nanoseconds * 1e-9
            if delta_sec > 0:
                current_fps = 1.0 / delta_sec
                self.arm_fps = 0.9 * self.arm_fps + 0.1 * current_fps  # Сглаживание
        
        self.arm_last_time = current_time
        
        cv2.putText(frame, f"FPS: {self.arm_fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        #cv2.imwrite(os.path.join(self.save_dir, 'arm_frame.jpg'), frame)
        cv2.imshow('Arm', resize(2, frame))
        cv2.waitKey(1)
        self.get_logger().info(f'Get Arm frame {frame.shape}')

    def binocular_display(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        current_time = self.get_clock().now()
        
        # Расчет FPS
        if self.binocular_last_time is not None:
            delta_time = current_time - self.binocular_last_time
            delta_sec = delta_time.nanoseconds * 1e-9
            if delta_sec > 0:
                current_fps = 1.0 / delta_sec
                self.binocular_fps = 0.9 * self.binocular_fps + 0.1 * current_fps
        
        self.binocular_last_time = current_time
        
        cv2.putText(frame, f"FPS: {self.binocular_fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        #cv2.imwrite(os.path.join(self.save_dir, 'binocular_frame.jpg'), frame)
        cv2.imshow('Binocular', resize(6, frame))
        cv2.waitKey(1)
        self.get_logger().info(f'Get Binocular frame {frame.shape}')

    def destroy_node(self):
        if self.out:
            self.out.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    cam_subscriber = CamerasSubscriber()
    try:
        rclpy.spin(cam_subscriber)
    except:
        cam_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()