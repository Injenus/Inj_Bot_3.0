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
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt


class MaxMinDetector:
    """
    Детектор паттерна:
      1) сначала расстояние растёт (до максимума),
      2) потом расстояние падает (до минимума),
      3) в момент минимума (перед стабилизацией/ростом) сигналим "готово".

    Параметр:
      stable_samples (N) — сколько последовательных измерений БЕЗ обновления
      максимума/минимума нужно, чтобы считать, что максимум/минимум пройден.
    """

    def __init__(self, stable_samples):
        self.stable_samples = stable_samples
        self.delta = 0.01
        self.reset()

    def reset(self):
        self.active = False
        self.stage = "IDLE"     # 'IDLE', 'UP', 'DOWN', 'FINISHED'
        self.current_max = None
        self.current_min = None
        self.max_stable = 0
        self.min_stable = 0

    def start(self, d0: float):
        """Запуск детектора для нового поворота."""
        self.reset()
        self.active = True
        self.stage = "UP"
        self.current_max = float(d0)
        self.current_min = float(d0)
        self.max_stable = 0
        self.min_stable = 0

    def update(self, d: float) -> str:
        """
        Обновление детектора.

        Возвращает:
          "none"      — ничего не произошло;
          "max_done"  — устойчивый максимум выявлен (переход в фазу DOWN);
          "min_done"  — устойчивый минимум выявлен (детектор закончил работу).
        """
        if not self.active:
            return "none"

        if d is None or math.isnan(d) or math.isinf(d):
            return "none"

        d = float(d)

        if self.stage == "UP":
            # Фаза роста: обновляем максимум, пока видим новые значения.
            if d >= self.current_max:
                self.current_max = d
                self.max_stable = 0
            else:
                self.max_stable += 1

            # если довольно давно не было нового максимума — считаем, что максимум пройден
            if self.max_stable >= self.stable_samples:
                # переходим к фазе DOWN
                self.stage = "DOWN"
                self.current_min = d
                self.min_stable = 0
                return "max_done"

            return "none"

        elif self.stage == "DOWN":
            # Фаза падения: обновляем минимум, пока видим новые значения.
            if d <= self.current_min:
                self.current_min = d
                self.min_stable = 0
            else:
                self.min_stable += 1

            # если довольно давно не было нового минимума — считаем, что минимум пройден
            if self.min_stable >= self.stable_samples:
                self.stage = "FINISHED"
                self.active = False
                return "min_done"

            return "none"

        return "none"


class MinMaxMinDetector:
    """
    Детектор паттерна для CCW-поворота:
      1) сначала расстояние падает (до первого минимума),
      2) потом растёт (до максимума),
      3) потом снова падает (до второго минимума),
      4) в момент второго минимума сигналим "готово".

    Интерфейс как у MaxMinDetector:
      update(d) -> "none" / "max_done" / "min_done"
    При этом:
      - первый минимум внутренний (можем логировать отдельно при желании),
      - "max_done" — после устойчивого максимума,
      - "min_done" — после устойчивого второго минимума.
    """

    def __init__(self, stable_samples: int):
        self.stable_samples = stable_samples
        self.reset()

    def reset(self):
        self.active = False
        # 'IDLE', 'DOWN1', 'UP', 'DOWN2', 'FINISHED'
        self.stage = "IDLE"

        self.min1 = None
        self.min1_stable = 0

        self.max_val = None
        self.max_stable = 0

        self.min2 = None
        self.min2_stable = 0

    def start(self, d0: float):
        self.reset()
        self.active = True
        self.stage = "DOWN1"
        d0 = float(d0)
        self.min1 = d0
        self.min1_stable = 0
        self.max_val = d0
        self.max_stable = 0
        self.min2 = d0
        self.min2_stable = 0

    def update(self, d: float) -> str:
        """
        Возвращает:
          "none"      — ничего не произошло;
          "max_done"  — устойчивый максимум (в середине паттерна);
          "min_done"  — устойчивый второй минимум (конец поворота).
        """
        if not self.active:
            return "none"

        if d is None or math.isnan(d) or math.isinf(d):
            return "none"

        d = float(d)

        # 1. Первая нисходящая фаза (поиск первого минимума)
        if self.stage == "DOWN1":
            if d <= self.min1:
                self.min1 = d
                self.min1_stable = 0
            else:
                self.min1_stable += 1

            if self.min1_stable >= self.stable_samples:
                # первый минимум найден, переходим к фазе роста
                self.stage = "UP"
                self.max_val = d
                self.max_stable = 0
                # первый минимум нам для внешней логики не нужен
                return "none"

            return "none"

        # 2. Фаза роста (поиск максимума)
        if self.stage == "UP":
            if d >= self.max_val:
                self.max_val = d
                self.max_stable = 0
            else:
                self.max_stable += 1

            if self.max_stable >= self.stable_samples:
                # максимум найден, переходим ко второму минимуму
                self.stage = "DOWN2"
                self.min2 = d
                self.min2_stable = 0
                return "max_done"

            return "none"

        # 3. Вторая нисходящая фаза (поиск второго минимума)
        if self.stage == "DOWN2":
            if d <= self.min2:
                self.min2 = d
                self.min2_stable = 0
            else:
                self.min2_stable += 1

            if self.min2_stable >= self.stable_samples:
                self.stage = "FINISHED"
                self.active = False
                return "min_done"

            return "none"

        return "none"


class RoomAlignNode(Node):
    """
    4 фазы движения:

      0) IDLE
      1) ROTATE_CW   — поворот ПО ЧАСОВОЙ до паттерна max→min по сглаженной фронтовой дистанции.
      2) FORWARD     — движение вперёд до target_front_dist.
      3) ROTATE_CCW  — поворот ПРОТИВ ЧАСОВОЙ по паттерну min→max→min.
      4) BACKWARD    — движение назад до target_back_dist.
      5) DONE        — завершение, статус 1, возврат в IDLE.
      6) ERROR       — ошибка, статус 2.
    """

    def __init__(self):
        super().__init__('move_to_start_from_center')

        # --------- Параметры движения ---------
        self.linear_speed = 0.12    # м/с
        self.angular_speed = 0.5    # рад/с

        self.target_front_dist = 0.30
        self.target_back_dist = 2.4
        self.distance_tolerance = 0.02

        # --------- Лидар ---------
        self.min_valid_range = 0.05
        self.max_valid_range = 6.0
        self.lidar_timeout = 10.

        # Окно по секторам для "переднего луча"
        self.front_window_size = 5  # 3 / 5 / 7 — нечётное
        if self.front_window_size % 2 == 0:
            self.front_window_size += 1
        self.front_window_half = (self.front_window_size - 1) // 2

        # EMA для сглаживания по времени
        self.filter_alpha = 0.4

        # Лимиты по времени
        self.max_rotate_time = 20.0
        self.max_forward_time = 15.0
        self.max_backward_time = 20.0

        # --------- Детекторы ---------
        self.perp_stable_samples = 4  # сколько раз подряд без обновления экстремума
        self.detector_cw = MaxMinDetector(stable_samples=self.perp_stable_samples)
        self.detector_ccw = MinMaxMinDetector(stable_samples=self.perp_stable_samples)
        self.detector_cw_started = False
        self.detector_ccw_started = False

        # --------- Машина состояний ---------
        self.was_idle = False
        self.STATE_IDLE        = 0
        self.pre_cw = True
        self.STATE_ROTATE_CW   = 1
        self.pre_forw = True
        self.STATE_FORWARD     = 2
        self.STATE_ROTATE_CCW  = 3
        self.pre_back = True
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

        # --------- Данные лидара ---------
        self.current_obstacles = {}     # {angle_deg: distance}
        self.last_lidar_time = 0.0
        self.last_front_spatial = None  # усреднённый по секторам "передний луч"
        self.front_filtered = None      # сглаженный по времени (EMA)

        # --------- Логи / графики ---------
        self.log_base_dir = os.path.expanduser('~/Inj_Bot_3.0/logs/room_align')
        os.makedirs(self.log_base_dir, exist_ok=True)

        self.run_active = False
        self.run_dir = None
        self.global_start_time = None
        self.phase_logs = {}  # {state_name: {'t': [], 'front': []}}

        # временные метки событий max/min для поворотов
        # время — относительно global_start_time
        self.rotate_events = {
            self.STATE_ROTATE_CW:   {'max': [], 'min': []},
            self.STATE_ROTATE_CCW:  {'max': [], 'min': []},
        }

        # --------- ROS-интерфейсы ---------
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
            100
        )

        self.status_pub = self.create_publisher(
            UInt8,
            'room_align_status',
            100
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 50 Гц
        self.dt_nominal = 0.02
        self.timer = self.create_timer(self.dt_nominal, self.main_loop)

        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)
        self.ready_pub.publish(String(data=self.get_name()))

        self.get_logger().info("RoomAlignNode (CW max→min, CCW min→max→min) initialized (IDLE)")

    # ------------------- Служебные методы -------------------

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

    # ------------------- Логирование -------------------

    def prepare_new_run_logging(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(self.log_base_dir, timestamp)
        os.makedirs(self.run_dir, exist_ok=True)

        self.global_start_time = time.time()
        self.phase_logs = {}
        for name in self.state_names.values():
            self.phase_logs[name] = {'t': [], 'front': []}

        # обнуляем события поворотов
        self.rotate_events = {
            self.STATE_ROTATE_CW:   {'max': [], 'min': []},
            self.STATE_ROTATE_CCW:  {'max': [], 'min': []},
        }

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

                # Отрисовка вертикальных линий событий для фаз поворота
                if state_name == self.state_names[self.STATE_ROTATE_CW]:
                    ev = self.rotate_events[self.STATE_ROTATE_CW]
                    # max — зелёные пунктирные
                    if ev['max']:
                        for x in ev['max']:
                            plt.axvline(x=x, linestyle='--', linewidth=1.2)
                    # min — красные сплошные
                    if ev['min']:
                        for x in ev['min']:
                            plt.axvline(x=x, linewidth=1.5)
                elif state_name == self.state_names[self.STATE_ROTATE_CCW]:
                    ev = self.rotate_events[self.STATE_ROTATE_CCW]
                    if ev['max']:
                        for x in ev['max']:
                            plt.axvline(x=x, linestyle='--', linewidth=1.2)
                    if ev['min']:
                        for x in ev['min']:
                            plt.axvline(x=x, linewidth=1.5)

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

    # ------------------- Callbacks -------------------

    def lidar_callback(self, msg: String):
        """
        Принимаем JSON: { "angle_deg": distance_m, ... }.
        Храним как self.current_obstacles.
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

        if not self.was_idle:
            self.get_logger().info(f"Start command received: {cmd}")
            self.start_algorithm()
            self.was_idle = True

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

    # ------------------- Передний луч -------------------

    def compute_front_distance_windowed(self):
        """
        "Передний луч" как среднее по front_window_size соседним секторам
        вокруг угла, который по кругу ближе всего к 0°.
        """
        obstacles = self.current_obstacles
        if not obstacles:
            return None

        keys = sorted(obstacles.keys())
        if not keys:
            return None

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

    # ------------------- Главный цикл -------------------

    def main_loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        if dt <= 0.0 or dt > 1.0:
            dt = self.dt_nominal
        self.last_loop_time = now

        # обновляем front_spatial и front_filtered
        front_spatial = self.compute_front_distance_windowed()
        if front_spatial is not None:
            self.last_front_spatial = front_spatial
            if self.front_filtered is None:
                filtered = front_spatial
            else:
                filtered = (
                    self.filter_alpha * front_spatial +
                    (1.0 - self.filter_alpha) * self.front_filtered
                )

            # квантование с шагом 0.005 м
            step = 0.005
            quant = round(filtered / step) * step
            # опционально — слегка округлить для красоты логов
            self.front_filtered = round(quant, 3)

        # лог для графиков
        self.log_front_distance(now)

        # проверка таймаута лидара в активных фазах
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
            # if self.pre_cw:
            #     self.publish_twist(0.2, 0.0)
            #     time.sleep(4.)
            #     self.publish_twist(0.0, 0.0)
            #     time.sleep(1.0)
            #     self.publish_twist(0.2, 0.0)
            #     time.sleep(1.)
            #     self.publish_twist(0.0, 0.0)
            #     time.sleep(0.2)
            #     self.pre_cw = False
            self.step_rotate_cw(now)

        elif self.state == self.STATE_FORWARD:
            if self.pre_forw:
                self.publish_twist(0.0, 1.2)
                time.sleep(0.55)
                self.publish_twist(0.0, 0.0)
                self.pre_forw = False
            self.step_forward(now)

        elif self.state == self.STATE_ROTATE_CCW:
            self.step_rotate_ccw(now)

        elif self.state == self.STATE_BACKWARD:
            if self.pre_back:
                self.publish_twist(0.0, -1.2)
                time.sleep(0.55)
                self.publish_twist(0.0, 0.0)
                self.pre_back = False
            self.step_backward(now)

        elif self.state == self.STATE_DONE:
            self.finish_run(True)
            self.set_state(self.STATE_IDLE)

        elif self.state == self.STATE_ERROR:
            self.publish_twist(0.0, 0.0)

    # ------------------- Этапы движения -------------------

    def step_rotate_cw(self, now: float):
        """
        ФАЗА 1: поворот ПО ЧАСОВОЙ.
        Останавливаемся по детектору max→min на front_filtered.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_rotate_time:
            self.get_logger().error("Timeout in ROTATE_CW")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        d = self.front_filtered
        if d is None:
            self.publish_twist(0.0, -self.angular_speed)
            return

        if not self.detector_cw_started:
            self.detector_cw.start(d)
            self.detector_cw_started = True
            self.get_logger().info(f"ROTATE_CW detector started at d0={d:.3f} m")

        event = self.detector_cw.update(d)

        if self.global_start_time is not None:
            t_rel = now - self.global_start_time
        else:
            t_rel = 0.0

        if event == "max_done":
            self.rotate_events[self.STATE_ROTATE_CW]['max'].append(t_rel)
            self.get_logger().info(
                f"ROTATE_CW: MAX confirmed at t={t_rel:.2f}s, d={d:.3f} m"
            )

        if event == "min_done":
            self.rotate_events[self.STATE_ROTATE_CW]['min'].append(t_rel)
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(
                f"ROTATE_CW: MIN confirmed (perpendicular) at t={t_rel:.2f}s, d={d:.3f} m"
            )
            self.set_state(self.STATE_FORWARD)
            return

        # продолжаем крутиться по часовой
        self.publish_twist(0.0, -self.angular_speed)

    def step_forward(self, now: float):
        """
        ФАЗА 2: движение вперёд до target_front_dist по front_filtered.
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
            lin_x = -self.linear_speed * 0.5
        else:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"FORWARD finished: d_front = {d:.3f} m")
            self.set_state(self.STATE_ROTATE_CCW)
            return

        self.publish_twist(lin_x, 0.0)

    def step_rotate_ccw(self, now: float):
        """
        ФАЗА 3: поворот ПРОТИВ ЧАСОВОЙ.
        Паттерн min→max→min по front_filtered.
        """
        elapsed = time.time() - self.state_start_time
        if elapsed > self.max_rotate_time:
            self.get_logger().error("Timeout in ROTATE_CCW")
            self.set_state(self.STATE_ERROR)
            self.finish_run(False)
            return

        d = self.front_filtered
        if d is None:
            self.publish_twist(0.0, self.angular_speed)
            return

        if not self.detector_ccw_started:
            self.detector_ccw.start(d)
            self.detector_ccw_started = True
            self.get_logger().info(f"ROTATE_CCW detector (min→max→min) started at d0={d:.3f} m")

        event = self.detector_ccw.update(d)

        if self.global_start_time is not None:
            t_rel = now - self.global_start_time
        else:
            t_rel = 0.0

        if event == "max_done":
            self.rotate_events[self.STATE_ROTATE_CCW]['max'].append(t_rel)
            self.get_logger().info(
                f"ROTATE_CCW: MAX confirmed at t={t_rel:.2f}s, d={d:.3f} m"
            )

        if event == "min_done":
            self.rotate_events[self.STATE_ROTATE_CCW]['min'].append(t_rel)
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(
                f"ROTATE_CCW: second MIN confirmed (perpendicular) at t={t_rel:.2f}s, d={d:.3f} m"
            )
            self.set_state(self.STATE_BACKWARD)
            return

        self.publish_twist(0.0, self.angular_speed)

    def step_backward(self, now: float):
        """
        ФАЗА 4: движение назад до target_back_dist по front_filtered.
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
