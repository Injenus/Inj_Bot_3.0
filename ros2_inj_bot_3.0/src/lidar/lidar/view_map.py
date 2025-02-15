import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1
        )
        self.max_radius = 5.0

        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})

        # 0 градусов строго вверх
        self.ax.set_theta_offset(np.pi / 2)
        self.ax.set_theta_direction(-1)  # Направление по часовой стрелке

        # Фиксированный масштаб
        self.ax.set_rmax(self.max_radius)
        self.ax.set_rlabel_position(-22.5)

        # Предполагаемый абсолютный диапазон интенсивности (замени на нужный)
        self.intensity_min = 0.0
        self.intensity_max = 47.0

        self.scatter = self.ax.scatter([], [], c=[], cmap="RdYlGn", s=10, vmin=self.intensity_min, vmax=self.intensity_max)
        self.ax.set_rlim(0, self.max_radius)
        plt.title("LIDAR Scan (Absolute Intensity Coloring)")
        plt.show(block=False)

    def scan_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        angles -= np.pi  # Поворот на 180 градусов (переворот лидара)

        ranges = np.array(msg.ranges)
        intensities = np.array(msg.intensities)
        #print(f'min= {np.min(intensities)} , max= {np.max(intensities)} , mean= {np.mean(intensities)} , median= {np.median(intensities)}')

        # Фильтр значений
        valid_mask = np.logical_and(
            np.isfinite(ranges),
            np.logical_and(
                ranges > msg.range_min,
                ranges < msg.range_max
            )
        )

        angles = angles[valid_mask]
        ranges = ranges[valid_mask]
        intensities = intensities[valid_mask]

        # Обрезаем интенсивность по заданным абсолютным границам
        intensities = np.clip(intensities, self.intensity_min, self.intensity_max)

        # Обновление графика
        self.scatter.remove()
        self.scatter = self.ax.scatter(angles, ranges, c=intensities, cmap="RdYlGn", s=10, vmin=self.intensity_min, vmax=self.intensity_max)
        self.ax.set_rlim(0, self.max_radius)

        # Перерисовка
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    visualizer = LidarVisualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
