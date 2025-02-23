import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import matplotlib.pyplot as plt
import threading

class PolarPlotter(Node):
    def __init__(self):
        super().__init__('polar_plotter')

        self.subscription = self.create_subscription(
            String,
            '/lidar/obstacles',
            self.obstacles_callback,
            1
        )

        # Включаем интерактивный режим Matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self.ax.set_ylim(0, 5)

        self.data_lock = threading.Lock()
        self.data = {}

        # Таймер для обновления графика в основном потоке
        self.timer = self.create_timer(0.1, self.update_plot)  # 10 Гц

        self.get_logger().info('Run ... ')

    def obstacles_callback(self, msg):
        """Обработка входящих данных."""
        with self.data_lock:
            self.data = json.loads(msg.data)

    def update_plot(self):
        """Обновление графика."""
        with self.data_lock:
            if not self.data:
                return

            # Сортировка ключей как чисел
            angles = np.array(sorted(self.data.keys(), key=lambda x: float(x)), dtype=np.float32)
            distances = np.array([self.data[str(int(angle))] for angle in angles], dtype=np.float32)

            if len(angles) < 2:
                return

            angle_step = np.abs(angles[1] - angles[0])
            half_step = angle_step / 2

            full_angles = []
            full_distances = []

            for i, center in enumerate(angles):
                lower = center - half_step
                upper = center + half_step

                angle_range = np.arange(lower, upper, 1)
                full_angles.extend(np.radians(angle_range))
                #full_angles.extend((np.radians(angle_range) + np.pi) % (2 * np.pi))  # +180°
                full_distances.extend([distances[i]] * len(angle_range))

            self.ax.clear()
            self.ax.set_theta_zero_location("N")
            self.ax.set_theta_direction(-1)
            self.ax.set_ylim(0, 5)

            self.ax.scatter(full_angles, full_distances, c='b', s=3)

            # Обновляем график
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    polar_plotter = PolarPlotter()
    try:
        rclpy.spin(polar_plotter)
    except KeyboardInterrupt:
        polar_plotter.destroy_node()
        rclpy.shutdown()
        plt.close('all')  # Закрываем все окна

if __name__ == '__main__':
    main()