import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from _custom_picamera2 import *
from _tools import *

class ArmCameraPublisher(Node):
    def __init__(self):
        super().__init__('arm_camera_publisher')
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'cam/arm', 1)
        self.publisher_grayscale = self.create_publisher(Image, 'cam/arm_g', 1)
        # 2591 x 1944  / 2.5 = 1036 x 778
        #2.15: 1205 x 904
        #4.2 617 x 463
        self.cam = Rpi_Camera(id=0, resolution=0, name='arm', hard_resize_koeff=4.2, rotate=180, hard_roi=None, calib_data=None, gains_roi=(0,0,1,1))
        
        self.timer = self.create_timer(0.02, self.publish_imge) # тянет только 15 фпс

    def publish_imge(self):
        self.cam.get_frame()
        if self.cam.frame is not None:
            #self.cam.frame = resize(4, self.cam.frame)
            
            image_msg = self.bridge.cv2_to_imgmsg(self.cam.frame, encoding='bgr8')
            self.publisher.publish(image_msg)

            gray_frame = cv2.cvtColor(self.cam.frame, cv2.COLOR_BGR2GRAY)
            gray_msg = self.bridge.cv2_to_imgmsg(gray_frame, 'mono8')
            self.publisher_grayscale.publish(gray_msg)

            self.get_logger().info('Published frames from Arm_Cam')

    
def main(args=None):
    rclpy.init(args=args)
    arm_cam = ArmCameraPublisher()

    rclpy.spin(arm_cam)

    arm_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


