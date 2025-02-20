import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

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
    
    def arm_display(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(os.path.join(self.save_dir, 'arm_frame.jpg'), frame)
        self.get_logger().info(f'Get Arm frame {frame.shape}')

    def binocular_display(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(os.path.join(self.save_dir, 'binocular_frame.jpg'), frame)
        self.get_logger().info(f'Get Binocular frame {frame.shape}')

def main(args=None):
    rclpy.init(args=args)
    cam_subscriber = CamerasSubscriber()
    try:
        rclpy.spin(cam_subscriber)
    except:
        cam_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()