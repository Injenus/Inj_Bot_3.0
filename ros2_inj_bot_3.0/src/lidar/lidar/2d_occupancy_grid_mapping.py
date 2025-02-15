import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.optimize import least_squares
from scipy.optimize import minimize
import threading

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # Параметры
        self.map_size = 15.0  # meters
        self.resolution = 0.05  # meters per grid cell
        self.icp_iterations = 20
        self.max_history = 1000  # максимальный размер истории
        
        # Инициализация положения
        self.pose = np.array([self.map_size/2, self.map_size/2, 0.0])  # x, y, theta
        
        # Структуры данных
        self.global_map = np.zeros((int(self.map_size/self.resolution), 
                                  int(self.map_size/self.resolution)))
        self.scan_history = []
        self.odometry = [self.pose.copy()]
        self.data_lock = threading.Lock()
        
        # Инициализация ROS
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Визуализация
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.map_img = self.ax.imshow(
            self.global_map, 
            cmap='gray_r', 
            origin='lower', 
            vmin=0, 
            vmax=1,
            extent=[0, self.map_size, 0, self.map_size]
        )
        self.robot_plot, = self.ax.plot([], [], 'ro', markersize=8)
        plt.title("Real-time SLAM Mapping")

    def scan_to_points(self, scan):
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges)
        valid = np.logical_and(
            ranges > scan.range_min,
            ranges < scan.range_max
        )
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        return np.column_stack((x, y))

    def icp_align(self, src, dst):
        def error_fn(transform):
            dx, dy, dtheta = transform
            rotation = np.array([
                [np.cos(dtheta), -np.sin(dtheta)],
                [np.sin(dtheta), np.cos(dtheta)]
            ])
            transformed = src @ rotation.T + [dx, dy]
            tree = KDTree(dst)
            dists, _ = tree.query(transformed)
            return np.mean(dists**2)

        result = minimize(
            error_fn,
            np.zeros(3),
            method='Powell',
            options={'maxiter': self.icp_iterations}
        )
        return result.x

    def update_map(self, points, pose):
        grid_coords = ((points + pose[:2]) / self.resolution).astype(int)
        valid = np.all(
            (grid_coords >= 0) & 
            (grid_coords < self.map_size/self.resolution), 
            axis=1
        )
        np.add.at(self.global_map, tuple(grid_coords[valid].T), 0.1)
        np.clip(self.global_map, 0, 1, out=self.global_map)

    def scan_callback(self, msg):
        current_points = self.scan_to_points(msg)
        
        with self.data_lock:
            # Обновление позиции через ICP
            if len(self.scan_history) > 0:
                prev_points = self.scan_history[-1]
                transform = self.icp_align(current_points, prev_points)
                self.pose += transform
            
            # Добавление новых данных
            self.scan_history.append(current_points)
            self.odometry.append(self.pose.copy())
            
            # Ограничение размера истории
            if len(self.scan_history) > self.max_history:
                self.scan_history.pop(0)
                self.odometry.pop(0)
            
            # Обновление карты
            rotation = np.array([
                [np.cos(self.pose[2]), -np.sin(self.pose[2])],
                [np.sin(self.pose[2]), np.cos(self.pose[2])]
            ])
            global_points = current_points @ rotation.T + self.pose[:2]
            self.update_map(global_points, self.pose)
        
        # Периодическая оптимизация
        if len(self.scan_history) % 10 == 0:
            self.global_loop_closure()
        
        self.visualize()

    def global_loop_closure(self):
        with self.data_lock:
            num_poses = len(self.odometry)
            if num_poses < 3:
                return

            # Оптимизация последних 20 сканов
            window_size = min(num_poses, 20)
            start_idx = max(0, num_poses - window_size)
            
            # Подготовка данных для оптимизации
            optimized_scans = self.scan_history[start_idx:]
            optimized_odom = self.odometry[start_idx:]
            
            # Построение матрицы связей
            edges = []
            for i in range(1, len(optimized_odom)):
                dx = optimized_odom[i][0] - optimized_odom[i-1][0]
                dy = optimized_odom[i][1] - optimized_odom[i-1][1]
                dtheta = optimized_odom[i][2] - optimized_odom[i-1][2]
                edges.append((i-1, i, np.array([dx, dy, dtheta])))

            def error_function(poses_flat):
                poses = poses_flat.reshape(-1, 3)
                err = []
                for edge in edges:
                    i, j, meas = edge
                    xi, yi, ti = poses[i]
                    xj, yj, tj = poses[j]
                    pred = np.array([xj - xi, yj - yi, tj - ti])
                    err.extend(pred - meas)
                return np.array(err)

            # Оптимизация
            try:
                result = least_squares(
                    error_function,
                    np.array(optimized_odom).flatten(),
                    method='trf',
                    ftol=1e-4,
                    max_nfev=50
                )
            except Exception as e:
                self.get_logger().error(f"Optimization failed: {e}")
                return

            # Обновление данных
            optimized_odom = result.x.reshape(-1, 3)
            self.odometry[start_idx:] = optimized_odom.tolist()
            self.pose = self.odometry[-1]
            
            # Полная перестройка карты
            self.global_map.fill(0)
            for i in range(len(self.odometry)):
                if i >= len(self.scan_history):
                    continue
                scan = self.scan_history[i]
                pose = self.odometry[i]
                rotation = np.array([
                    [np.cos(pose[2]), -np.sin(pose[2])],
                    [np.sin(pose[2]), np.cos(pose[2])]
                ])
                global_points = scan @ rotation.T + pose[:2]
                self.update_map(global_points, pose)

    def visualize(self):
        self.map_img.set_data(self.global_map)
        self.robot_plot.set_data([self.pose[0]], [self.pose[1]])
        self.ax.set_xlim(0, self.map_size)
        self.ax.set_ylim(0, self.map_size)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    slam_node = SLAMNode()
    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()