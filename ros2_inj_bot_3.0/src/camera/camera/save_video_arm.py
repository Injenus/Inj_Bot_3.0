import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
from datetime import datetime
import sys

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from _tools import *

saving_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0')
os.makedirs(saving_path, exist_ok=True)
now_moment = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")

class ArmSavingVideo(Node):
    def __init__(self):
        super().__init__("arm_saving_video")
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, 'cam/arm', self.display_n_save, 1)

        self.video_path = os.path.join(saving_path, f"arm_cam_{now_moment}.avi")
        self.get_logger().info(f"Path is {self.video_path}")

        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.frame_rate = 15
        self.out = None
        self.timer = time.time()

    def display_n_save(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.out is None:
            h, w = frame.shape[:2]
            self.out = cv2.VideoWriter(self.video_path, self.fourcc, self.frame_rate, (w, h))

        self.out.write(frame)

        # cv2.imshow("Arm", resize(2, frame))
        # cv2.waitKey(1)

        self.get_logger().info('Got arm frame')

    def destroy_node(self):
        if self.out:
            self.out.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    arm_video = ArmSavingVideo()
    try:
        rclpy.spin(arm_video)
    except:
        arm_video.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        