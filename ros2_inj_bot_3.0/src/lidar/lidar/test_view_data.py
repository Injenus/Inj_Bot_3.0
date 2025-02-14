import rclpy
from rclpy.node import Node
import numpy as np
from rplidar import RPLidar
import matplotlib.pyplot as plt
from threading import Thread, Lock
from queue import Queue

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        
        # Инициализация лидара
        self.lidar = RPLidar('/dev/ttyUSB0')
        self.lidar.start_motor()
        
        # Потокобезопасные структуры
        self.data_queue = Queue(maxsize=1)
        self.lock = Lock()
        
        # Запуск потока сбора данных
        self.scan_thread = Thread(target=self.read_lidar)
        self.scan_thread.daemon = True
        self.scan_thread.start()
        
        # Настройка графики
        self.fig, self.ax = plt.subplots()
        self.timer = self.create_timer(0.05, self.update_plot)  # ROS2 Timer вместо plt.pause()

    def read_lidar(self):
        if 1:
            for scan in self.lidar.iter_scans():
                points = []
                for (_, angle, distance) in scan:
                    if distance <= 0 or distance > 5000:  # Фильтрация
                        continue
                    rad = np.radians(angle % 360)  # Нормализация угла
                    x = distance * np.cos(rad)
                    y = distance * np.sin(rad)
                    points.append((x, y))
                
                # Безопасная запись в очередь
                with self.lock:
                    if self.data_queue.full():
                        self.data_queue.get_nowait()
                    self.data_queue.put_nowait(points)

        # except Exception as e:
        #     self.get_logger().error(f"Lidar error: {str(e)}")
        # finally:
        #     self.lidar.stop_motor()
        #     self.lidar.disconnect()

    def update_plot(self):
        try:
            with self.lock:
                if not self.data_queue.empty():
                    points = self.data_queue.get_nowait()
                    
                    # Отрисовка
                    self.ax.clear()
                    if points:
                        x = [p[0] for p in points]
                        y = [p[1] for p in points]
                        self.ax.scatter(x, y, s=1, c='blue')
                    
                    # Настройки графика
                    self.ax.set_xlim(-5000, 5000)
                    self.ax.set_ylim(-5000, 5000)
                    self.ax.set_aspect('equal')
                    self.fig.canvas.draw_idle()
                    plt.pause(0.001)  # Необходимо для обновления GUI
                    
        except Exception as e:
            self.get_logger().error(f"Plot error: {str(e)}")

    def destroy_node(self):
        # Корректное завершение
        self.lidar.stop_motor()
        self.lidar.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    visualizer = LidarVisualizer()
    
    try:
        plt.show(block=False)
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()