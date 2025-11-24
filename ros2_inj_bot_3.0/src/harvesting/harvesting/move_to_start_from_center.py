import rclpy
from rclpy.node import Node

from std_msgs.msg import String, UInt8
from geometry_msgs.msg import Twist

import json
import math
import time
import os
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt


def wrap_to_pi(angle: float) -> float:
    """
    Привести угол к диапазону [-pi, +pi].
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RoomAlignNode(Node):
    """
    Узел выравнивания робота по стене по данным из топика 'lidar/obstacles'.

    Интерфейс:
    - Подписки:
        * 'lidar/obstacles' (std_msgs/String, JSON: "angle_deg" -> distance_m)
        * 'room_align_cmd' (std_msgs/UInt8): 0 = сброс, !=0 = старт алгоритма
    - Публикации:
        * '/cmd_vel' (geometry_msgs/Twist)
        * 'room_align_status' (UInt8): 1 = успех, 2 = ошибка
        * '/nodes_ready' (String) — объявление готовности узла

    Алгоритм:
    1) ALIGN: замкнутое управление поворотом по нормали к стене (оценка через PCA).
    2) FORWARD: движение к стене до target_front_dist, при этом сохраняем перпендикулярность.
    3) BACKWARD: отъезд назад до target_back_dist, также с удержанием перпендикулярности.
    4) DONE/ERROR: остановка, публикация статуса, сохранение графиков логов.
    """

    def __init__(self):
        super().__init__('move_to_start_from_center')

        # --- Параметры движения и геометрии ---
        self.linear_speed = 0.1              # м/с
        self.angular_speed_max = 0.4          # рад/с (максимальная угловая скорость)
        self.angular_kp = 1.5                 # коэффициент P-регулятора по углу

        self.target_front_dist = 0.30         # цель: расстояние спереди после подъезда к стене
        self.target_back_dist = 2.10          # цель: расстояние спереди после отъезда назад
        self.distance_tolerance = 0.02        # допуск по расстоянию (2 см)

        self.min_valid_range = 0.05           # минимальная валидная дальность
        self.max_valid_range = 6.0            # максимальная валидная дальность для контроля
        self.max_fit_range = 4.0              # максимально используемое расстояние для оценки ориентации

        self.front_fov_deg = 30.0             # половина сектора перед роботом, где ищем стену
        self.min_points_for_fit = 5           # минимальное число точек для PCA
        self.max_line_rms = 0.15              # максимально допустимое RMS-отклонение точек от найденной линии (м)
        self.angle_tolerance_rad = math.radians(2.0)  # точность выравнивания по углу (≈2°)

        self.filter_alpha = 0.3               # сглаживание расстояния спереди (EMA)
        self.lidar_timeout = 5.0              # таймаут данных лидара (с)

        # Ограничения по времени этапов (с)
        self.max_align_time = 12.0
        self.max_forward_time = 10.0
        self.max_backward_time = 12.0

        # --- Состояния машины ---
        self.STATE_IDLE = 0
        self.STATE_ALIGN = 1
        self.STATE_FORWARD = 2
        self.STATE_BACKWARD = 3
        self.STATE_DONE = 4
        self.STATE_ERROR = 5

        self.state_names = {
            self.STATE_IDLE: "IDLE",
            self.STATE_ALIGN: "ALIGN",
            self.STATE_FORWARD: "FORWARD",
            self.STATE_BACKWARD: "BACKWARD",
            self.STATE_DONE: "DONE",
            self.STATE_ERROR: "ERROR",
        }

        self.state = self.STATE_IDLE
        self.state_start_time = time.time()

        # --- Данные лидара ---
        self.current_obstacles = {}  # {angle_deg (int): distance (float)}
        self.front_raw = None
        self.front_filtered = None
        self.last_lidar_time = 0.0

        # --- Логирование для графиков ---
        self.log_base_dir = os.path.expanduser('~/Inj_Bot_3.0/logs/room_align')
        os.makedirs(self.log_base_dir, exist_ok=True)
        self.run_active = False
        self.run_dir = None
        self.global_start_time = None
        self.phase_logs = {}  # {state_name: {'t': [], 'front': []}}

        # --- ROS интерфейсы ---
        self.lidar_sub = self.create_subscription(
            String,
            'lidar/obstacles',
            self.lidar_callback,
            10
        )

        self.cmd_sub = self.create_subscription(
            UInt8,
            'room_align_cmd',
            self.command_callback,
            10
        )

        self.status_pub = self.create_publisher(
            UInt8,
            'room_align_status',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # таймер основного цикла
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.main_loop)

        # публикация готовности
        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)
        self.ready_pub.publish(String(data=self.get_name()))

        self.get_logger().info("RoomAlignNode initialized (IDLE)")

    # ------------------------------------------------------------
    # Вспомогательные методы для логики
    # ------------------------------------------------------------

    def set_state(self, new_state: int):
        """
        Смена состояния с логированием и остановкой движения.
        """
        if new_state not in self.state_names:
            self.get_logger().warn(f"Unknown state code: {new_state}")
        self.state = new_state
        self.state_start_time = time.time()
        self.publish_twist(0.0, 0.0)

        self.get_logger().info(f"STATE: {self.state_names.get(new_state, 'UNKNOWN')}")

    def lidar_is_fresh(self) -> bool:
        return (time.time() - self.last_lidar_time) <= self.lidar_timeout

    def publish_twist(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.cmd_vel_pub.publish(msg)

    def publish_status(self, code: int):
        msg = UInt8()
        msg.data = int(code)
        self.status_pub.publish(msg)

    # ------------------------------------------------------------
    # Логирование и сохранение графиков
    # ------------------------------------------------------------

    def prepare_new_run_logging(self):
        """
        Создать директорию для нового запуска и обнулить буферы логов.
        """
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(self.log_base_dir, timestamp)
        os.makedirs(self.run_dir, exist_ok=True)

        self.global_start_time = time.time()
        self.phase_logs = {}
        for name in self.state_names.values():
            self.phase_logs[name] = {'t': [], 'front': []}

        self.run_active = True
        self.get_logger().info(f"Logging for run in '{self.run_dir}'")

    def log_front_distance(self):
        """
        Записать точку (t, front_filtered) для текущего состояния.
        """
        if not self.run_active:
            return
        if self.front_filtered is None or self.global_start_time is None:
            return

        state_name = self.state_names.get(self.state, "UNKNOWN")
        t_rel = time.time() - self.global_start_time

        self.phase_logs[state_name]['t'].append(t_rel)
        self.phase_logs[state_name]['front'].append(self.front_filtered)

    def save_plots(self):
        """
        Сохранить графики front_filtered(t) для каждого состояния, где есть данные.
        """
        if not self.run_active or self.run_dir is None:
            return

        for state_name, logs in self.phase_logs.items():
            t = logs['t']
            d = logs['front']
            if len(t) < 2:
                continue

            try:
                plt.figure()
                plt.plot(t, d, marker='o', linestyle='-')
                plt.xlabel('t, s')
                plt.ylabel('front distance, m')
                plt.title(f'RoomAlign: {state_name}')
                plt.grid(True)

                filename = os.path.join(self.run_dir, f'front_{state_name}.jpg')
                plt.savefig(filename, dpi=150)
                plt.close()
                self.get_logger().info(f"Saved plot for state {state_name}: {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save plot for {state_name}: {e}")

        self.run_active = False
        self.run_dir = None
        self.global_start_time = None

    def finish_run(self, success: bool):
        """
        Общий финализатор запуска: остановка, статус, сохранение графиков.
        """
        self.publish_twist(0.0, 0.0)
        self.publish_status(1 if success else 2)
        self.save_plots()

    # ------------------------------------------------------------
    # Обработка входящих сообщений
    # ------------------------------------------------------------

    def lidar_callback(self, msg: String):
        """
        Приём JSON с секторами: { "angle_deg": distance_m, ... }.
        angle_deg — целое (0..359), направление: 0° — вперёд, >0 — против часовой стрелки.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("lidar_callback: JSON decode error")
            return

        if not isinstance(data, dict):
            return

        obstacles = {}
        front_dist = None

        for k, v in data.items():
            try:
                angle_deg = int(k)
                dist = float(v)
            except (ValueError, TypeError):
                continue

            if math.isnan(dist) or math.isinf(dist):
                continue

            if dist < self.min_valid_range or dist > self.max_valid_range:
                continue

            obstacles[angle_deg] = dist

            if angle_deg == 0:
                front_dist = dist

        if not obstacles:
            return

        self.current_obstacles = obstacles
        self.last_lidar_time = time.time()

        if front_dist is not None:
            self.front_raw = front_dist
            if self.front_filtered is None:
                self.front_filtered = front_dist
            else:
                self.front_filtered = (
                    self.filter_alpha * front_dist +
                    (1.0 - self.filter_alpha) * self.front_filtered
                )

    def command_callback(self, msg: UInt8):
        """
        Команда управления алгоритмом.
        0   — сброс в IDLE.
        !=0 — запуск алгоритма (если в IDLE).
        """
        cmd = msg.data

        if cmd == 0:
            self.get_logger().info("Received reset command (0)")
            self.set_state(self.STATE_IDLE)
            # сброс логирования текущего запуска не сохраняем — как есть
            return

        if self.state != self.STATE_IDLE:
            self.get_logger().info(f"Command {cmd} ignored: state != IDLE")
            return

        self.get_logger().info(f"Start command received: {cmd}")
        self.start_algorithm()

    def start_algorithm(self):
        """
        Проверка готовности лидара и запуск алгоритма выравнивания.
        """
        now = time.time()
        if (self.front_filtered is None) or ((now - self.last_lidar_time) > self.lidar_timeout):
            self.get_logger().error("Cannot start: no recent lidar data")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        self.prepare_new_run_logging()
        self.set_state(self.STATE_ALIGN)

    # ------------------------------------------------------------
    # Оценка ориентации стены
    # ------------------------------------------------------------

    def estimate_wall_orientation_error(self):
        """
        Оценить ошибку ориентации относительно стены.

        Использует точки в секторе [-front_fov_deg, +front_fov_deg],
        переводит их в координаты (x, y) и делает PCA (SVD), чтобы
        получить ориентацию линии стены и, соответственно, нормали к ней.

        Возвращает:
            error_angle (float, рад) в диапазоне [-pi/2, pi/2] — угол нормали стены
            относительно оси робота (0 рад).
            Или None, если оценка ненадёжна.
        """
        if not self.current_obstacles:
            return None

        angles = []
        dists = []

        for angle_deg, dist in self.current_obstacles.items():
            # нормируем угол в [-180, 180]
            a_norm = ((angle_deg + 180) % 360) - 180
            if abs(a_norm) > self.front_fov_deg:
                continue

            if dist < self.min_valid_range or dist > self.max_fit_range:
                continue

            angles.append(float(a_norm))
            dists.append(float(dist))

        if len(angles) < self.min_points_for_fit:
            return None

        angles_rad = np.deg2rad(np.array(angles, dtype=float))
        dists_arr = np.array(dists, dtype=float)

        # Перевод в декартовы координаты в СК робота
        x = dists_arr * np.cos(angles_rad)
        y = dists_arr * np.sin(angles_rad)

        pts = np.stack((x, y), axis=1)
        center = np.mean(pts, axis=0)
        X = pts - center

        # SVD для поиска главной оси
        try:
            _, _, Vt = np.linalg.svd(X, full_matrices=False)
        except np.linalg.LinAlgError:
            return None

        direction = Vt[0, :]  # вектор вдоль линии стены (единичный)

        # Нормаль к стене (перпендикуляр к direction)
        normal = np.array([-direction[1], direction[0]], dtype=float)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-6:
            return None
        normal /= norm_len

        # Проверка качества аппроксимации линии (RMS отклонение)
        distances_to_line = X @ normal  # проекция на нормаль
        rms = float(np.sqrt(np.mean(distances_to_line ** 2)))
        if rms > self.max_line_rms:
            # Точек много, но они не ложатся на одну линию — вероятно, не одна стена
            return None

        # Угол линии и нормали
        theta_line = math.atan2(direction[1], direction[0])
        theta_normal = wrap_to_pi(theta_line + math.pi / 2.0)

        # У нормали два возможных направления, выбираем с меньшим |углом|
        theta_normal_alt = wrap_to_pi(theta_normal + math.pi)
        if abs(theta_normal_alt) < abs(theta_normal):
            theta_normal = theta_normal_alt

        # Теперь theta_normal — угол нормали стены в СК робота.
        # Ошибка ориентации = theta_normal (хотим 0).
        # Гарантируем диапазон [-pi/2, pi/2] для устойчивого управления.
        theta_normal = wrap_to_pi(theta_normal)
        if theta_normal > math.pi / 2:
            theta_normal -= math.pi
        elif theta_normal < -math.pi / 2:
            theta_normal += math.pi

        return theta_normal

    # ------------------------------------------------------------
    # Основной цикл и этапы
    # ------------------------------------------------------------

    def main_loop(self):
        # Логируем фронт каждую итерацию (если есть данные и запущен алгоритм)
        self.log_front_distance()

        # Контроль таймаута лидара во время активных этапов
        if self.state in [self.STATE_ALIGN, self.STATE_FORWARD, self.STATE_BACKWARD]:
            if not self.lidar_is_fresh():
                self.get_logger().error("Lidar data timeout — stopping and switching to ERROR")
                self.set_state(self.STATE_ERROR)
                self.finish_run(False)
                return

        if self.state == self.STATE_IDLE:
            # Узел ничего не делает, движения нет
            self.publish_twist(0.0, 0.0)

        elif self.state == self.STATE_ALIGN:
            self.step_align()

        elif self.state == self.STATE_FORWARD:
            self.step_forward()

        elif self.state == self.STATE_BACKWARD:
            self.step_backward()

        elif self.state == self.STATE_DONE:
            # Первый вход в DONE завершает запуск
            self.finish_run(True)
            self.set_state(self.STATE_IDLE)

        elif self.state == self.STATE_ERROR:
            # В состоянии ошибки просто стоим
            self.publish_twist(0.0, 0.0)

    def step_align(self):
        """
        Этап выравнивания по ориентации: вращаемся, пока нормаль к стене не станет ≈0 рад.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_align_time:
            self.get_logger().error("Timeout in ALIGN")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        angle_error = self.estimate_wall_orientation_error()

        if angle_error is None:
            # Пока ориентация не оценена, медленно крутимся, чтобы собрать больше точек.
            self.publish_twist(0.0, 0.2)
            return

        if abs(angle_error) < self.angle_tolerance_rad:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"ALIGN finished: angle_error={math.degrees(angle_error):.2f} deg")
            self.set_state(self.STATE_FORWARD)
            return

        # P-регулятор по углу
        w = self.angular_kp * angle_error
        w = max(-self.angular_speed_max, min(self.angular_speed_max, w))
        self.publish_twist(0.0, w)

    def step_forward(self):
        """
        Этап движения вперёд к стене до target_front_dist.
        Одновременно удерживаем ориентацию по стене.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_forward_time:
            self.get_logger().error("Timeout in FORWARD")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        d = self.front_filtered
        if d is None:
            self.publish_twist(0.0, 0.0)
            return

        # Контроль ориентации
        angle_error = self.estimate_wall_orientation_error()
        ang_z = 0.0
        if angle_error is not None:
            if abs(angle_error) > self.angle_tolerance_rad / 2.0:
                ang_z = self.angular_kp * angle_error
                ang_z = max(-self.angular_speed_max, min(self.angular_speed_max, ang_z))

        # Контроль расстояния
        target = self.target_front_dist
        tol = self.distance_tolerance

        if d > (target + tol):
            lin_x = self.linear_speed
        elif d < (target - tol):
            # слишком близко — чуть отъезжаем назад
            lin_x = -self.linear_speed * 0.5
        else:
            # цель по расстоянию достигнута
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"FORWARD finished: d_front = {d:.3f} m")
            self.set_state(self.STATE_BACKWARD)
            return

        self.publish_twist(lin_x, ang_z)

    def step_backward(self):
        """
        Этап движения назад до целевого расстояния target_back_dist.
        Также удерживаем перпендикуляр к стене.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_backward_time:
            self.get_logger().error("Timeout in BACKWARD")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        d = self.front_filtered
        if d is None:
            self.publish_twist(0.0, 0.0)
            return

        # Контроль ориентации
        angle_error = self.estimate_wall_orientation_error()
        ang_z = 0.0
        if angle_error is not None:
            if abs(angle_error) > self.angle_tolerance_rad / 2.0:
                ang_z = self.angular_kp * angle_error
                ang_z = max(-self.angular_speed_max, min(self.angular_speed_max, ang_z))

        target = self.target_back_dist
        tol = self.distance_tolerance

        # Хотим: d_front >= target - tol (то есть достаточно далеко от стены)
        if d < (target - tol):
            lin_x = -self.linear_speed
        else:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"BACKWARD finished: d_front = {d:.3f} m")
            self.set_state(self.STATE_DONE)
            return

        self.publish_twist(lin_x, ang_z)


def main(args=None):
    rclpy.init(args=args)
    node = RoomAlignNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.save_plots()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
