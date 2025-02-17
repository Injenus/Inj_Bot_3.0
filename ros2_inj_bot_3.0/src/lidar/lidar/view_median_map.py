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

        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_theta_zero_location("N")  # 0° направлено вверх
        self.ax.set_theta_direction(-1)  # Углы идут против часовой стрелки
        self.ax.set_ylim(0, 10)  # Задаем радиус карты (можно изменить)

        self.data_lock = threading.Lock()
        self.data = {}

        # Запускаем поток обновления графика
        self.plot_thread = threading.Thread(target=self.update_plot)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def obstacles_callback(self, msg):
        """ Обрабатывает входящие данные и обновляет локальный буфер. """
        with self.data_lock:
            self.data = json.loads(msg.data)

    def update_plot(self):
        """ Цикл обновления графика. """
        while rclpy.ok():
            with self.data_lock:
                if not self.data:
                    continue

                angles = np.array(sorted(self.data.keys()), dtype=np.float32)
                distances = np.array([self.data[str(int(a))] for a in angles], dtype=np.float32)

                if len(angles) < 2:
                    continue

                # Вычисляем `angle_step`
                angle_step = np.abs(angles[1] - angles[0])
                half_step = angle_step / 2

                # Создаем новые массивы углов и расстояний для полярного графика
                full_angles = []
                full_distances = []

                for i, center in enumerate(angles):
                    lower = center - half_step
                    upper = center + half_step

                    angle_range = np.arange(lower, upper, 1)  # Заполняем 1-градусными шагами
                    full_angles.extend(np.radians(angle_range))
                    full_distances.extend([distances[i]] * len(angle_range))

                self.ax.clear()
                self.ax.set_theta_zero_location("N")
                self.ax.set_theta_direction(-1)
                self.ax.set_ylim(0, np.nanmax(distances) * 1.2 if np.any(np.isfinite(distances)) else 10)

                self.ax.scatter(full_angles, full_distances, c='b', s=10)
                plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    polar_plotter = PolarPlotter()

    try:
        rclpy.spin(polar_plotter)
    except:
        polar_plotter.destroy_node()
        rclpy.shutdown()
        plt.close()

if __name__ == '__main__':
    main()
