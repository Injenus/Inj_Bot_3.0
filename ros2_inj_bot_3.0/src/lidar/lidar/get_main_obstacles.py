import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import json
import os
import sys

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'pharma_delivery', 'pharma_delivery')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config_04_2025 as conf

class LidarObstacles(Node):
    def __init__(self):
        super().__init__('lidar_obstacles')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            3
        )

        self.publisher = self.create_publisher(String, 'lidar/obstacles', 3)
        self.angle_step = conf.LIDAR_STEP 
        self.declare_parameter('smoothing_factor', 0.8)  # EMA smoothing factor (0-1)
        self.smoothed_medians = {}  # Stores smoothed values per sector
        self.get_logger().info('Run ... ')

        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)
        msg = String(data = self.get_name())
        self.ready_pub.publish(msg)

    def scan_callback(self, msg):

        smoothing_factor = self.get_parameter('smoothing_factor').get_parameter_value().double_value
        if not (0 < smoothing_factor <= 1):
            self.get_logger().error("smoothing_factor must be between 0 and 1!")
            return

        # Calculate angles with 180° rotation correction
        angles_raw = np.degrees(msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment)
        rotated_angles = (angles_raw + 180) % 360
        distances = np.array(msg.ranges)
        half_step = self.angle_step / 2

        # Process sectors
        centers = np.arange(0, 360, self.angle_step)
        current_medians = {}

        for center in centers:
            # Calculate sector boundaries
            lower = (center - half_step) % 360
            upper = (center + half_step) % 360

            # Create angle mask
            if lower < upper:
                mask = (rotated_angles >= lower) & (rotated_angles < upper)
            else:
                mask = (rotated_angles >= lower) | (rotated_angles < upper)

            sector_data = np.where(np.isinf(distances[mask]), 3.5, distances[mask])  # Замена inf на 5.0

            clean_data = sector_data[~np.isnan(sector_data)]
            
            if clean_data.size > 0:
                median = np.median(clean_data)
            else:
                median = float('nan')
            
            current_medians[round(center)] = float(median)

        # Apply EMA smoothing
        smoothed_medians = {}
        for center, median in current_medians.items():
            if np.isfinite(median):
                if center in self.smoothed_medians:
                    prev = self.smoothed_medians[center]
                    smoothed = smoothing_factor * median + (1 - smoothing_factor) * prev
                else:
                    smoothed = median
                smoothed_medians[center] = smoothed
            else:
                # Preserve last valid value if available
                smoothed_medians[center] = self.smoothed_medians.get(center, np.nan)

        self.smoothed_medians = smoothed_medians

        # Publish results
        msg_out = String()
        msg_out.data = json.dumps(smoothed_medians)
        #print(msg_out.data)
        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    lidar_obstacles = LidarObstacles()
    rclpy.spin(lidar_obstacles)
    lidar_obstacles.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()