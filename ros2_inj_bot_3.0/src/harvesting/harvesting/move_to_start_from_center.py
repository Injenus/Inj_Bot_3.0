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

import matplotlib
matplotlib.use("Agg")  # без GUI
import matplotlib.pyplot as plt


def wrap_to_pi(angle: float) -> float:
    """Привести угол к диапазону [-pi, +pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RoomAlignNode(Node):
    """
    Алгоритм движения:

    0) IDLE        — ожидание команды.
    1) ROTATE_CW   — поворот по часовой, пока нормаль к стене ≈ 0 рад.
                     При этом накапливаем угол вращения angle_rotated_cw.
    2) FORWARD     — езда вперёд до target_front_dist.
    3) ROTATE_CCW  — поворот против часовой на тот же угол angle_rotated_cw.
    4) BACKWARD    — езда назад до target_back_dist.
    5) DONE        — конец, статус 1, возврат в IDLE.
    6) ERROR       — ошибка, статус 2.

    Вход:
      - 'lidar/obstacles' (String, JSON: "angle_deg" -> distance_m),
      - 'room_align_cmd' (UInt8): 0 = сброс, !=0 = старт.

    Выход:
      - '/cmd_vel' (Twist),
      - 'room_align_status' (UInt8: 1 — успех, 2 — ошибка),
      - '/nodes_ready' (String).
    """

    def __init__(self):
        super().__init__('move_to_start_from_center')

        # ---------------- Параметры движения ----------------
        self.linear_speed = 0.15               # м/с вперёд/назад
        self.angular_speed = 0.6               # рад/с (минимум, чтобы робот реально крутился)
        self.angular_corr_max = 0.3            # макс. корректирующая угловая скорость при езде

        self.target_front_dist = 0.30          # цель: передняя дистанция после подъезда к стене
        self.target_back_dist = 1.10           # цель: передняя дистанция после отъезда назад
        self.distance_tolerance = 0.02         # допуск по дистанции (2 см)

        # ---------------- Геометрия / оценка стены ----------------
        self.min_valid_range = 0.05            # минимальная валидная дальность
        self.max_valid_range = 6.0             # максимальная валидная дальность (фильтр)
        self.max_fit_range = 4.0               # макс. расстояние точек для оценки стены

        self.front_fov_deg = 60.0              # половина фронтального сектора для оценки ориентации
        self.min_points_for_fit = 6            # минимум точек для PCA
        self.max_line_rms = 0.20               # максимум RMS-ошибки аппроксимации стеной (м)
        self.angle_tolerance_rad = math.radians(3.0)  # точность по нормали к стене (≈3°)

        # ---------------- Общие параметры ----------------
        self.filter_alpha = 0.3                # сглаживание front_filtered
        self.lidar_timeout = 1.0               # таймаут данных лидара (с)

        self.max_rotate_time = 20.0            # максимум времени на каждый поворот (с)
        self.max_forward_time = 15.0           # максимум времени на езду вперёд
        self.max_backward_time = 20.0          # максимум времени на езду назад
        self.max_rotate_angle = 2.0 * math.pi  # лимит угла на этап ROTATE_CW, рад

        # ---------------- Машина состояний ----------------
        self.STATE_IDLE = 0
        self.STATE_ROTATE_CW = 1
        self.STATE_FORWARD = 2
        self.STATE_ROTATE_CCW = 3
        self.STATE_BACKWARD = 4
        self.STATE_DONE = 5
        self.STATE_ERROR = 6

        self.state_names = {
            self.STATE_IDLE: "IDLE",
            self.STATE_ROTATE_CW: "ROTATE_CW",
            self.STATE_FORWARD: "FORWARD",
            self.STATE_ROTATE_CCW: "ROTATE_CCW",
            self.STATE_BACKWARD: "BACKWARD",
            self.STATE_DONE: "DONE",
            self.STATE_ERROR: "ERROR",
        }

        self.state = self.STATE_IDLE
        self.state_start_time = time.time()
        self.last_loop_time = time.time()

        # Для восстановления ориентации
        self.angle_rotated_cw = 0.0            # накопленный угол на этапе ROTATE_CW
        self.angle_rotated_ccw = 0.0           # накопленный угол на этапе ROTATE_CCW

        # ---------------- Данные лидара ----------------
        self.current_obstacles = {}            # {angle_deg: distance}
        self.front_raw = None
        self.front_filtered = None
        self.last_lidar_time = 0.0

        # ---------------- Логи и графики ----------------
        self.log_base_dir = os.path.expanduser('~/Inj_Bot_3.0/logs/room_align')
        os.makedirs(self.log_base_dir, exist_ok=True)

        self.run_active = False
        self.run_dir = None
        self.global_start_time = None
        self.phase_logs = {}  # {state_name: {'t': [], 'front': []}}

        # ---------------- ROS интерфейсы ----------------
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

        self.dt_nominal = 0.05
        self.timer = self.create_timer(self.dt_nominal, self.main_loop)

        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)
        self.ready_pub.publish(String(data=self.get_name()))

        self.get_logger().info("RoomAlignNode initialized (IDLE)")

    # ---------------- Вспомогательные методы ----------------

    def set_state(self, new_state: int):
        """Смена состояния с логом и остановкой движения."""
        if new_state not in self.state_names:
            self.get_logger().warn(f"Unknown state code: {new_state}")
        self.state = new_state
        self.state_start_time = time.time()

        # При входе в повороты сбрасываем накопленные углы
        if new_state == self.STATE_ROTATE_CW:
            self.angle_rotated_cw = 0.0
        elif new_state == self.STATE_ROTATE_CCW:
            self.angle_rotated_ccw = 0.0

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

    # ---------------- Логирование и графики ----------------

    def prepare_new_run_logging(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(self.log_base_dir, timestamp)
        os.makedirs(self.run_dir, exist_ok=True)

        self.global_start_time = time.time()
        self.phase_logs = {}
        for name in self.state_names.values():
            self.phase_logs[name] = {'t': [], 'front': []}

        self.run_active = True
        self.get_logger().info(f"Logging for run in '{self.run_dir}'")

    def log_front_distance(self, t_now: float):
        if not self.run_active:
            return
        if self.front_filtered is None or self.global_start_time is None:
            return

        state_name = self.state_names.get(self.state, "UNKNOWN")
        t_rel = t_now - self.global_start_time

        self.phase_logs[state_name]['t'].append(t_rel)
        self.phase_logs[state_name]['front'].append(self.front_filtered)

    def save_plots(self):
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
        """Останавливаемся, публикуем статус и сохраняем графики."""
        self.publish_twist(0.0, 0.0)
        self.publish_status(1 if success else 2)
        self.save_plots()

    # ---------------- Callbacks ----------------

    def lidar_callback(self, msg: String):
        """
        Сообщение от узла LidarObstacles:
          JSON: { "angle_deg": distance_m, ... }
        0° — вперёд.
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
        cmd = msg.data

        if cmd == 0:
            self.get_logger().info("Received reset command (0)")
            self.set_state(self.STATE_IDLE)
            # текущий лог завершаем без успешного завершения
            return

        if self.state != self.STATE_IDLE:
            self.get_logger().info(f"Command {cmd} ignored: state != IDLE")
            return

        self.get_logger().info(f"Start command received: {cmd}")
        self.start_algorithm()

    def start_algorithm(self):
        now = time.time()
        if (self.front_filtered is None) or ((now - self.last_lidar_time) > self.lidar_timeout):
            self.get_logger().error("Cannot start: no recent lidar data")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        self.prepare_new_run_logging()
        self.set_state(self.STATE_ROTATE_CW)

    # ---------------- Оценка ориентации стены ----------------

    def estimate_wall_orientation_error(self):
        """
        Возвращает угол нормали к стене (рад) в СК робота (0 рад — перпендикуляр спереди),
        или None, если оценка ненадёжна.
        """
        if not self.current_obstacles:
            return None

        angles = []
        dists = []

        for angle_deg, dist in self.current_obstacles.items():
            # приводим угол к [-180, 180]
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

        x = dists_arr * np.cos(angles_rad)
        y = dists_arr * np.sin(angles_rad)

        pts = np.stack((x, y), axis=1)
        center = np.mean(pts, axis=0)
        X = pts - center

        try:
            _, _, Vt = np.linalg.svd(X, full_matrices=False)
        except np.linalg.LinAlgError:
            return None

        direction = Vt[0, :]  # направление стены
        normal = np.array([-direction[1], direction[0]], dtype=float)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-6:
            return None
        normal /= norm_len

        # Проверяем качество "линейности" стены
        distances_to_line = X @ normal
        rms = float(np.sqrt(np.mean(distances_to_line ** 2)))
        if rms > self.max_line_rms:
            return None

        theta_line = math.atan2(direction[1], direction[0])
        theta_normal = wrap_to_pi(theta_line + math.pi / 2.0)

        # Берём направление нормали с меньшим |углом|
        theta_normal_alt = wrap_to_pi(theta_normal + math.pi)
        if abs(theta_normal_alt) < abs(theta_normal):
            theta_normal = theta_normal_alt

        theta_normal = wrap_to_pi(theta_normal)
        if theta_normal > math.pi / 2:
            theta_normal -= math.pi
        elif theta_normal < -math.pi / 2:
            theta_normal += math.pi

        return theta_normal

    # ---------------- Основной цикл ----------------

    def main_loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        if dt <= 0.0 or dt > 1.0:
            dt = self.dt_nominal
        self.last_loop_time = now

        # Лог дистанции по 0°
        self.log_front_distance(now)

        # Контроль таймаута лидара в активных состояниях
        if self.state in [self.STATE_ROTATE_CW, self.STATE_FORWARD,
                          self.STATE_ROTATE_CCW, self.STATE_BACKWARD]:
            if not self.lidar_is_fresh():
                self.get_logger().error("Lidar timeout — ERROR")
                self.set_state(self.STATE_ERROR)
                self.finish_run(False)
                return

        if self.state == self.STATE_IDLE:
            self.publish_twist(0.0, 0.0)

        elif self.state == self.STATE_ROTATE_CW:
            self.step_rotate_cw(dt)

        elif self.state == self.STATE_FORWARD:
            self.step_forward(dt)

        elif self.state == self.STATE_ROTATE_CCW:
            self.step_rotate_ccw(dt)

        elif self.state == self.STATE_BACKWARD:
            self.step_backward(dt)

        elif self.state == self.STATE_DONE:
            self.finish_run(True)
            self.set_state(self.STATE_IDLE)

        elif self.state == self.STATE_ERROR:
            self.publish_twist(0.0, 0.0)

    # ---------------- Этапы алгоритма ----------------

    def step_rotate_cw(self, dt: float):
        """
        2. Вращение ПО ЧАСОВОЙ стрелке (angular.z < 0)
           до тех пор, пока нормаль стены ≈ 0 рад.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_rotate_time:
            self.get_logger().error("Timeout in ROTATE_CW")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        if self.angle_rotated_cw > self.max_rotate_angle:
            self.get_logger().error("Max angle exceeded in ROTATE_CW")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        angle_error = self.estimate_wall_orientation_error()

        if angle_error is not None:
            self.get_logger().debug(
                f"ROTATE_CW: angle_error={math.degrees(angle_error):.2f} deg"
            )
            if abs(angle_error) <= self.angle_tolerance_rad:
                # считаем, что перпендикуляр найден
                self.publish_twist(0.0, 0.0)
                self.get_logger().info(
                    f"Perpendicular found (CW): angle_error={math.degrees(angle_error):.2f} deg, "
                    f"angle_rotated_cw={math.degrees(self.angle_rotated_cw):.2f} deg"
                )
                # сохраняем угол для последующего CCW
                # (ориентация при старте -> перпендикуляр)
                self.set_state(self.STATE_FORWARD)
                return

        # продолжаем крутиться по часовой
        w = -self.angular_speed
        self.publish_twist(0.0, w)
        self.angle_rotated_cw += abs(w) * dt

    def step_forward(self, dt: float):
        """
        3. Езда вперёд до target_front_dist.
           Дополнительно можем слегка подравнивать ориентацию по стене.
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

        # маленькая корректировка ориентации по стене
        angle_error = self.estimate_wall_orientation_error()
        ang_z = 0.0
        if angle_error is not None:
            if abs(angle_error) > self.angle_tolerance_rad / 2.0:
                ang_z = max(
                    -self.angular_corr_max,
                    min(self.angular_corr_max, 1.5 * angle_error)
                )

        target = self.target_front_dist
        tol = self.distance_tolerance

        if d > (target + tol):
            lin_x = self.linear_speed
        elif d < (target - tol):
            # слишком близко — чуть сдаём назад
            lin_x = -self.linear_speed * 0.5
        else:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"FORWARD finished: d_front = {d:.3f} m")
            self.set_state(self.STATE_ROTATE_CCW)
            return

        self.publish_twist(lin_x, ang_z)

    def step_rotate_ccw(self, dt: float):
        """
        4. Вращение ПРОТИВ ЧАСОВОЙ стрелки на тот же угол, что и в ROTATE_CW.
           Здесь мы не ищем перпендикуляр; цель — восстановить исходную ориентацию.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_rotate_time:
            self.get_logger().error("Timeout in ROTATE_CCW")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        # Если в ROTATE_CW по какой-то причине не накопили угол (очень странно),
        # просто сразу переходим к BACKWARD.
        if self.angle_rotated_cw <= 0.0:
            self.get_logger().warn("angle_rotated_cw <= 0, skipping ROTATE_CCW")
            self.set_state(self.STATE_BACKWARD)
            return

        if self.angle_rotated_ccw >= self.angle_rotated_cw:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(
                f"ROTATE_CCW finished: angle_rotated_ccw={math.degrees(self.angle_rotated_ccw):.2f} deg"
            )
            self.set_state(self.STATE_BACKWARD)
            return

        w = self.angular_speed  # ПРОТИВ часовой
        self.publish_twist(0.0, w)
        self.angle_rotated_ccw += abs(w) * dt

    def step_backward(self, dt: float):
        """
        5. Езда назад до target_back_dist.
           Можем слегка контролировать ориентацию по стене.
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

        angle_error = self.estimate_wall_orientation_error()
        ang_z = 0.0
        if angle_error is not None:
            if abs(angle_error) > self.angle_tolerance_rad / 2.0:
                ang_z = max(
                    -self.angular_corr_max,
                    min(self.angular_corr_max, 1.5 * angle_error)
                )

        target = self.target_back_dist
        tol = self.distance_tolerance

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
