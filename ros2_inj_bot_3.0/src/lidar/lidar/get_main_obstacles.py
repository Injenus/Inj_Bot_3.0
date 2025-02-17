import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import json

class LidarObstacles(Node):
    def __init__(self):
        super().__init__('lidar_obstacles')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1
        )

        self.publisher = self.create_publisher(String, '/lidar/obstacles', 1)

        self.angle_step = 20  # Настроечный параметр (ширина интервалов)
        self.half_step = self.angle_step / 2

    def scan_callback(self, msg):
        angles = np.linspace(np.degrees(msg.angle_min), np.degrees(msg.angle_max), len(msg.ranges))
        distances = np.array(msg.ranges)

        # Определение центров интервалов, чтобы один центр был 0°
        bins = [0]

        # Добавляем интервалы влево
        angle = -self.half_step
        while angle >= np.min(angles):
            bins.insert(0, angle)
            angle -= self.angle_step

        # Добавляем интервалы вправо
        angle = self.half_step
        while angle <= np.max(angles):
            bins.append(angle)
            angle += self.angle_step

        # Проверка краев
        if bins[0] > np.min(angles):  # Если есть недостающий кусок слева
            bins.insert(0, (bins[0] + np.min(angles)) / 2)
        if bins[-1] < np.max(angles):  # Если есть недостающий кусок справа
            bins.append((bins[-1] + np.max(angles)) / 2)

        # Вычисление медианы расстояний в каждом интервале
        median_values = {}

        for center in bins:
            lower = center - self.half_step
            upper = center + self.half_step

            mask = (angles >= lower) & (angles < upper)
            valid_distances = distances[mask]

            if len(valid_distances) > 0:
                median_values[round(center)] = float(np.median(valid_distances))
            else:
                median_values[round(center)] = float('nan')

        msg_out = String()
        msg_out.data = json.dumps(median_values)
        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    lidar_obstacles = LidarObstacles()
    try:
        rclpy.spin(lidar_obstacles)
    except:
        lidar_obstacles.destroy_node()
        rclpy.shutdown()
      
if __name__ == '__main__':
    main()
