import rclpy
from rclpy.node import Node

from std_msgs.msg import String, UInt8
from geometry_msgs.msg import Twist

import json
import math
import time
import os
from datetime import datetime

import matplotlib
matplotlib.use("Agg")  # без GUI
import matplotlib.pyplot as plt


class OneBeamPerpDetector:
    """
    Однолучевой детектор перпендикуляра.

    Работает по скаляру d(t) — расстояние "вперёд".

    Логика:
      - На старте запоминаем d0.
      - Сначала ждём, пока |d - d0| >= leave_threshold -> ушли от старой стены.
      - После этого ищем ЛОКАЛЬНЫЙ МИНИМУМ:
          * best_d — наименьшее из увиденных,
          * как только d > best_d + delta_eps несколько раз подряд
            (min_increase_samples) — считаем, что минимум пройден -> перпендикуляр.
    """

    def __init__(self,
                 leave_threshold: float = 0.08,
                 delta_eps: float = 0.005,
                 min_increase_samples: int = 4):
        self.leave_threshold = leave_threshold
        self.delta_eps = delta_eps
        self.min_increase_samples = min_increase_samples
        self.reset()

    def reset(self):
        self.running = False
        self.initial_d = None
        self.left_initial = False
        self.best_d = None
        self.increase_count = 0

    def start(self, initial_d: float):
        self.running = True
        self.initial_d = float(initial_d)
        self.left_initial = False
        self.best_d = float('inf')
        self.increase_count = 0

    def update(self, d: float) -> bool:
        """
        Обновить детектор новым значением d.
        Возвращает True, когда найден локальный минимум.
        """
        if not self.running:
            return False
        if d is None or math.isnan(d) or math.isinf(d):
            return False

        # Ещё не ушли от начальной стены
        if not self.left_initial:
            if abs(d - self.initial_d) >= self.leave_threshold:
                self.left_initial = True
                self.best_d = d
            return False

        # Уже ушли от исходного положения -> ищем минимум
        if d < self.best_d - self.delta_eps:
            # нашли более глубокий минимум
            self.best_d = d
            self.increase_count = 0
            return False
        elif d > self.best_d + self.delta_eps:
            # пошли вверх от минимума
            self.increase_count += 1
            if self.increase_count >= self.min_increase_samples:
                # минимум пройден -> считаем перпендикуляр достигнутым
                return True
        else:
            # в окрестности минимума, но без уверенного роста
            self.increase_count = 0

        return False


class RoomAlignNode(Node):
    """
    4 фазы движения:

    0) IDLE
    1) ROTATE_CW   — вращение по часовой до перпендикуляра (по однолучевому детектору).
    2) FORWARD     — движение вперёд до target_front_dist.
    3) ROTATE_CCW  — вращение против часовой до перпендикуляра.
    4) BACKWARD    — движение назад до target_back_dist.
    5) DONE        — завершение, статус 1, возврат в IDLE.
    6) ERROR       — ошибка, статус 2.

    Данные:
      - 'lidar/obstacles' (String, JSON: "angle_deg" -> distance_m, 0° — вперёд),
      - 'room_align_cmd' (UInt8): 0 = сброс, !=0 = запуск.

    Управление:
      - '/cmd_vel' (Twist),
      - 'room_align_status' (UInt8),
      - '/nodes_ready' (String).

    Логирование:
      - для каждого состояния отдельный график front_filtered(t):
        ~/Inj_Bot_3.0/logs/room_align/<timestamp>/front_<STATE>.jpg
    """

    def __init__(self):
        super().__init__('move_to_start_from_center')

        # ------------ Параметры движения ------------
        self.linear_speed = 0.15                # м/с
        self.angular_speed = 0.6                # рад/с (по модулю)

        self.target_front_dist = 0.30           # цель после подъезда к стене
        self.target_back_dist = 1.10            # цель после отъезда назад
        self.distance_tolerance = 0.02          # допуск по расстоянию

        # ------------ Параметры лидара ------------
        self.min_valid_range = 0.05             # минимальная валидная дальность
        self.max_valid_range = 6.0              # максимальная валидная дальность
        self.lidar_timeout = 0.5                # таймаут данных лидара

        # Окно по секторам для "переднего луча"
        self.front_window_size = 3              # 3, 5 или 7 (нечётное)
        if self.front_window_size % 2 == 0:
            self.front_window_size += 1
        self.front_window_half = (self.front_window_size - 1) // 2

        # Сглаживание по времени для логов и контроля расстояния
        self.filter_alpha = 0.4                 # EMA для front_filtered

        # Ограничения по времени этапов
        self.max_rotate_time = 10.0             # макс. время поворота (с)
        self.max_forward_time = 15.0            # макс. время движения вперёд (с)
        self.max_backward_time = 20.0           # макс. время движения назад (с)

        # ------------ Детекторы перпендикуляра ------------
        self.perp_leave_threshold = 0.08        # сколько уйти от d0, чтобы начать искать новый минимум
        self.perp_delta_eps = 0.005             # порог различимости по расстоянию (м)
        self.perp_min_increase_samples = 4      # сколько шагов роста после минимума

        self.detector_cw = OneBeamPerpDetector(
            leave_threshold=self.perp_leave_threshold,
            delta_eps=self.perp_delta_eps,
            min_increase_samples=self.perp_min_increase_samples
        )
        self.detector_ccw = OneBeamPerpDetector(
            leave_threshold=self.perp_leave_threshold,
            delta_eps=self.perp_delta_eps,
            min_increase_samples=self.perp_min_increase_samples
        )
        self.detector_cw_started = False
        self.detector_ccw_started = False

        # ------------ Машина состояний ------------
        self.STATE_IDLE        = 0
        self.STATE_ROTATE_CW   = 1
        self.STATE_FORWARD     = 2
        self.STATE_ROTATE_CCW  = 3
        self.STATE_BACKWARD    = 4
        self.STATE_DONE        = 5
        self.STATE_ERROR       = 6

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

        # ------------ Данные лидара ------------
        self.current_obstacles = {}             # {angle_deg: distance}
        self.last_lidar_time = 0.0

        self.last_front_spatial = None          # усреднённый по секторам передний луч (без EMA)
        self.front_filtered = None              # EMA по времени для логов/контроля

        # ------------ Логи / графики ------------
        self.log_base_dir = os.path.expanduser('~/Inj_Bot_3.0/logs/room_align')
        os.makedirs(self.log_base_dir, exist_ok=True)

        self.run_active = False
        self.run_dir = None
        self.global_start_time = None
        self.phase_logs = {}                    # {state_name: {'t': [], 'front': []}}

        # ------------ ROS интерфейсы ------------
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

        self.dt_nominal = 0.01
        self.timer = self.create_timer(self.dt_nominal, self.main_loop)

        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)
        self.ready_pub.publish(String(data=self.get_name()))

        self.get_logger().info("RoomAlignNode (one-beam) initialized (IDLE)")

    # ------------ Вспомогательные методы ------------

    def set_state(self, new_state: int):
        if new_state not in self.state_names:
            self.get_logger().warn(f"Unknown state code: {new_state}")
        self.state = new_state
        self.state_start_time = time.time()
        self.publish_twist(0.0, 0.0)

        if new_state == self.STATE_ROTATE_CW:
            self.detector_cw.reset()
            self.detector_cw_started = False
        elif new_state == self.STATE_ROTATE_CCW:
            self.detector_ccw.reset()
            self.detector_ccw_started = False

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

    # ------------ Логирование и графики ------------

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
        self.publish_twist(0.0, 0.0)
        self.publish_status(1 if success else 2)
        self.save_plots()

    # ------------ Обработка сообщений ------------

    def lidar_callback(self, msg: String):
        """
        Принимаем JSON: { "angle_deg": distance_m, ... }.
        Сохраняем как self.current_obstacles без обработки 0° здесь.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("lidar_callback: JSON decode error")
            return

        if not isinstance(data, dict):
            return

        obstacles = {}
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

        if not obstacles:
            return

        self.current_obstacles = obstacles
        self.last_lidar_time = time.time()

    def command_callback(self, msg: UInt8):
        cmd = msg.data

        if cmd == 0:
            self.get_logger().info("Received reset command (0)")
            self.set_state(self.STATE_IDLE)
            return

        if self.state != self.STATE_IDLE:
            self.get_logger().info(f"Command {cmd} ignored: state != IDLE")
            return

        self.get_logger().info(f"Start command received: {cmd}")
        self.start_algorithm()

    def start_algorithm(self):
        if not self.lidar_is_fresh() or not self.current_obstacles:
            self.get_logger().error("Cannot start: no recent lidar data")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        initial_d = self.compute_front_distance_windowed()
        if initial_d is None:
            self.get_logger().error("Cannot start: no valid front distance")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        self.last_front_spatial = initial_d
        self.front_filtered = initial_d

        self.prepare_new_run_logging()
        self.set_state(self.STATE_ROTATE_CW)

    # ------------ Передний луч (усреднение по секторам) ------------

    def compute_front_distance_windowed(self):
        """
        Возвращает расстояние "вперёд" как среднее
        по front_window_size соседним секторам вокруг 0°.

        Берём сектор, угол которого ближе всего к 0° по кругу,
        и несколько соседних по ИНДЕКСУ (не по самому значению угла).
        """
        obstacles = self.current_obstacles
        if not obstacles:
            return None

        keys = sorted(obstacles.keys())  # например, [0, 10, 20, ..., 350]
        if not keys:
            return None

        # функция "насколько близко к 0°" по кругу
        def circular_abs(a: int) -> float:
            a_norm = ((a + 180) % 360) - 180
            return abs(a_norm)

        front_key = min(keys, key=circular_abs)
        front_index = keys.index(front_key)

        values = []
        n = len(keys)
        for offset in range(-self.front_window_half, self.front_window_half + 1):
            j = (front_index + offset) % n
            angle = keys[j]
            val = obstacles.get(angle, None)
            if val is None:
                continue
            if math.isnan(val) or math.isinf(val):
                continue
            values.append(val)

        if not values:
            return None

        return float(sum(values) / len(values))

    # ------------ Основной цикл ------------

    def main_loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        if dt <= 0.0 or dt > 1.0:
            dt = self.dt_nominal
        self.last_loop_time = now

        # Обновляем передний луч и EMA
        front_spatial = self.compute_front_distance_windowed()
        if front_spatial is not None:
            self.last_front_spatial = front_spatial
            if self.front_filtered is None:
                self.front_filtered = front_spatial
            else:
                self.front_filtered = (
                    self.filter_alpha * front_spatial +
                    (1.0 - self.filter_alpha) * self.front_filtered
                )

        # Логируем front_filtered
        self.log_front_distance(now)

        # Контроль таймаута лидара в активных фазах
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
            self.step_rotate_cw()

        elif self.state == self.STATE_FORWARD:
            self.step_forward()

        elif self.state == self.STATE_ROTATE_CCW:
            self.step_rotate_ccw()

        elif self.state == self.STATE_BACKWARD:
            self.step_backward()

        elif self.state == self.STATE_DONE:
            self.finish_run(True)
            self.set_state(self.STATE_IDLE)

        elif self.state == self.STATE_ERROR:
            self.publish_twist(0.0, 0.0)

    # ------------ Этапы движения ------------

    def step_rotate_cw(self):
        """
        ФАЗА 1: Поворот ПО ЧАСОВОЙ до перпендикуляра по однолучевому детектору.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_rotate_time:
            self.get_logger().error("Timeout in ROTATE_CW")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        d = self.last_front_spatial
        if d is None:
            # нет дистанции — крутимся в надежде, что данные появятся
            self.publish_twist(0.0, -self.angular_speed)
            return

        if not self.detector_cw_started:
            self.detector_cw.start(d)
            self.detector_cw_started = True
            self.get_logger().info(f"ROTATE_CW detector started at d0={d:.3f} m")

        if self.detector_cw.update(d):
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"Perpendicular (CW) detected at d_front={d:.3f} m")
            self.set_state(self.STATE_FORWARD)
            return

        # продолжаем крутиться ПО ЧАСОВОЙ
        self.publish_twist(0.0, -self.angular_speed)

    def step_forward(self):
        """
        ФАЗА 2: Движение вперёд до target_front_dist.
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

        target = self.target_front_dist
        tol = self.distance_tolerance

        if d > (target + tol):
            lin_x = self.linear_speed
        elif d < (target - tol):
            lin_x = -self.linear_speed * 0.5  # чуть отъехать, если переехали
        else:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"FORWARD finished: d_front = {d:.3f} m")
            self.set_state(self.STATE_ROTATE_CCW)
            return

        self.publish_twist(lin_x, 0.0)

    def step_rotate_ccw(self):
        """
        ФАЗА 3: Поворот ПРОТИВ ЧАСОВОЙ до перпендикуляра по однолучевому детектору.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_rotate_time:
            self.get_logger().error("Timeout in ROTATE_CCW")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        d = self.last_front_spatial
        if d is None:
            self.publish_twist(0.0, self.angular_speed)
            return

        if not self.detector_ccw_started:
            self.detector_ccw.start(d)
            self.detector_ccw_started = True
            self.get_logger().info(f"ROTATE_CCW detector started at d0={d:.3f} m")

        if self.detector_ccw.update(d):
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"Perpendicular (CCW) detected at d_front={d:.3f} m")
            self.set_state(self.STATE_BACKWARD)
            return

        # продолжаем крутиться ПРОТИВ ЧАСОВОЙ
        self.publish_twist(0.0, self.angular_speed)

    def step_backward(self):
        """
        ФАЗА 4: Движение назад до target_back_dist.
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

        target = self.target_back_dist
        tol = self.distance_tolerance

        if d < (target - tol):
            lin_x = -self.linear_speed
        else:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"BACKWARD finished: d_front = {d:.3f} m")
            self.set_state(self.STATE_DONE)
            return

        self.publish_twist(lin_x, 0.0)


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
