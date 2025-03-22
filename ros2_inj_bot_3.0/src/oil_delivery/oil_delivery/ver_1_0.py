"""
 Движение по запсанным логам, начало чтения логов - старт движения.
 
"""
import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import copy
import time
from rclpy.node import Node
from rosbag2_py import Player, StorageOptions, PlayOptions
import subprocess
import signal
from std_msgs.msg import UInt8MultiArray

current_script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(current_script_path)
print(script_dir)
log_patterns_path = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..', '..', 'src', 'log_patterns'))
print(log_patterns_path)


class Oil_Delivery(Node):
    def __init__(self):
        super().__init__('oil_delivery')

        self.servo_pub = self.create_publisher(UInt8MultiArray, 'servo/to_write', 1)


        # self.launch_child_nodes()

        self.main_line()

        # self.child_processes = []
        #self._start_bag_playback(os.path.join(log_patterns_path, '/example'))


        

    
    def _start_bag_playback(self, bag_path):
        cmd = [
            'ros2', 'bag', 'play',
            bag_path,
        ]
        
        try:
            self.bag_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
            self.get_logger().info(f"Playback started with PID: {self.bag_proc.pid}")
        except Exception as e:
            self.get_logger().error(f"Failed to start playback: {str(e)}")
            raise


    def main_line(self):
        #self._start_bag_playback(f"{os.path.join(log_patterns_path, 'example')}")

        def servo_pub(data):
            servo_msg = UInt8MultiArray()
            servo_msg.data = data
            self.servo_pub.publish(servo_msg)

        # servo_pub(bytes([142, 133, 130, 132, 0, 128, 42]))
        # time.sleep(3.0)
        # Измените путь на реальный абсолютный путь к вашим логам
        time.sleep(3.0)
        servo_pub(bytes([142-90, 133+75, 130-120, 132, 0, 128, 42]))

        time.sleep(5.0)
        servo_pub(bytes([142, 133, 130, 132, 0, 128, 42]))
        





    def destroy_node(self):
        # self.get_logger().info("Stopping, destroy..")
        # for proc in self.child_processes:
        #     if proc.poll() is None:
        #         os.killing(os.getpid(proc.pid), signal.SIGTERM)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Oil_Delivery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
