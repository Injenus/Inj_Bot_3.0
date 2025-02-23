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
        self.declare_parameter('angle_step', 10)  # Шаг секторов (должен делить 360)
        self.get_logger().info('Run ... ')

    def scan_callback(self, msg):
        # Проверка что angle_step делит 360
        angle_step = self.get_parameter('angle_step').get_parameter_value().integer_value
        if 360 % angle_step != 0:
            self.get_logger().error(f"angle_step {angle_step} must be a divisor of 360!")
            return

        # Рассчитываем углы с поворотом на 180° для коррекции ориентации лидара
        angles_raw = np.degrees(msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment)
        rotated_angles = (angles_raw + 180) % 360  # Корректируем направление
        
        distances = np.array(msg.ranges)
        half_step = angle_step / 2

        # Генерируем центры секторов [0, angle_step, 2*angle_step, ...]
        centers = np.arange(0, 360, angle_step)
        median_values = {}

        for center in centers:
            # Вычисляем границы сектора с учетом перехода через 0°
            lower = (center - half_step) % 360
            upper = (center + half_step) % 360

            # Формируем маску для углов
            if lower < upper:
                mask = (rotated_angles >= lower) & (rotated_angles < upper)
            else:
                # Сектор пересекает 0°
                mask = (rotated_angles >= lower) | (rotated_angles < upper)

            valid_distances = distances[mask]
            median = np.median(valid_distances) if valid_distances.size > 0 else float('nan')
            median_values[round(center)] = float(median)

        # Публикация результатов
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