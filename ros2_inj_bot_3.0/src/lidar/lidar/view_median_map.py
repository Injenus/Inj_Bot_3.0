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
            'lidar/obstacles',
            self.obstacles_callback,
            3
        )

        self.data_lock = threading.Lock()
        self.data = {}
        self.range = 5.0
        
        # Инициализация графика
        plt.ion()  # Интерактивный режим
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(6, 6))
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self.ax.set_ylim(0, self.range)
        self.fig.show()

        # Таймер для обновления графика
        self.timer = self.create_timer(0.02, self.update_plot)

    def obstacles_callback(self, msg):
        with self.data_lock:
            self.data = json.loads(msg.data)
            #self.get_logger().info(f'{self.data}')

    def update_plot(self):
        with self.data_lock:
            if not self.data:
                return

            self.ax.clear()

            self.ax.set_theta_zero_location("N")
            self.ax.set_theta_direction(-1)
            self.ax.set_ylim(0, self.range)
            
            angles = np.array([int(k) for k in self.data.keys()])
            distances = np.array([float(v) for v in self.data.values()])
            angle_step = abs(angles[0]-angles[1])

            # Все точки синим
            self.ax.scatter(np.radians(angles), distances, c='b', s=1)

            red_angles = []
            red_distances = []
            yellow_angles = []
            yellow_distances = []

            # Целевые углы (0, 90, 180, 270)
            target_angles = [0, 90, 180, 270]
            for angle in target_angles:
                # Проверяем основной угол
                key = f"{int(angle)}"  # Предполагаем целые ключи типа "0", "90"
                if key in self.data:
                    red_angles.append(np.radians(angle))
                    red_distances.append(float(self.data[key]))
                
                # Проверяем соседей
                for delta in [-angle_step, angle_step]:
                    neighbor_angle = angle + delta
                    neighbor_key = f"{int(neighbor_angle % 360)}"
                    if neighbor_key in self.data:
                        yellow_angles.append(np.radians(neighbor_angle))
                        yellow_distances.append(float(self.data[neighbor_key]))

            # Отрисовка цветных точек
            if red_angles:
                self.ax.scatter(red_angles, red_distances, c='r', s=3, zorder=3)
            if yellow_angles:
                self.ax.scatter(yellow_angles, yellow_distances, c='yellow', s=6, zorder=2)
            
            # Обновление графика
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

if __name__ == '__main__':
    main()