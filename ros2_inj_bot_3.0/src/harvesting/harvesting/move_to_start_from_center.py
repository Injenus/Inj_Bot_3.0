import rclpy
from rclpy.node import Node

from std_msgs.msg import String, UInt8
from geometry_msgs.msg import Twist

import json
import math
import time


class PerpendicularDetector:
    def __init__(self, delta_eps: float = 0.01, min_stable_samples: int = 3):
        self.delta_eps = delta_eps
        self.min_stable_samples = min_stable_samples
        self.reset()

    def reset(self):
        self.prev_value = None
        self.pos_count = 0
        self.neg_count = 0
        self.corner_seen = False
        self.trend = 'UNKNOWN'  # 'UNKNOWN', 'UP', 'DOWN'

    def update(self, value: float) -> bool:
        if value is None:
            return False

        if self.prev_value is None:
            self.prev_value = value
            return False

        delta = value - self.prev_value
        self.prev_value = value

        if delta > self.delta_eps:
            sign = 1
        elif delta < -self.delta_eps:
            sign = -1
        else:
            sign = 0

        if sign == 1:
            self.pos_count += 1
            self.neg_count = 0
        elif sign == -1:
            self.neg_count += 1
            self.pos_count = 0
        else:
            if self.pos_count > 0:
                self.pos_count -= 1
            if self.neg_count > 0:
                self.neg_count -= 1

        N = self.min_stable_samples

        if not self.corner_seen:
            if self.trend != 'UP' and self.pos_count >= N:
                self.trend = 'UP'
            elif self.trend == 'UP' and self.neg_count >= N:
                self.trend = 'DOWN'
                self.corner_seen = True
        else:
            if self.trend != 'DOWN' and self.neg_count >= N:
                self.trend = 'DOWN'
            elif self.trend == 'DOWN' and self.pos_count >= N:
                # локальный минимум
                return True

        return False


class RoomAlignNode(Node):
    def __init__(self):
        super().__init__('move_to_start_from_center')

        # --- Параметры движения / фильтра ---
        self.linear_speed = 0.15
        self.angular_speed = 0.4

        self.target_front_dist = 0.30
        self.distance_tolerance = 0.02

        # новый шаг: целевая дистанция спереди после отъезда назад
        self.target_back_dist = 2.10

        self.filter_alpha = 0.3
        self.min_valid_range = 0.05
        self.max_valid_range = 6.0

        self.delta_eps = 0.01
        self.min_stable_samples = 3

        self.max_rotate_time = 12.0
        self.max_forward_time = 10.0
        self.max_backward_time = 12.0

        # --- состояние лидара ---
        self.front_raw = None
        self.front_filtered = None
        self.last_lidar_time = 0.0
        self.lidar_timeout = 5.5

        # --- машина состояний ---
        self.STATE_IDLE = 0
        self.STATE_ROTATE_CW = 1
        self.STATE_FORWARD = 2
        self.STATE_ROTATE_CCW = 3
        self.STATE_BACKWARD = 4
        self.STATE_DONE = 5
        self.STATE_ERROR = 6

        self.state = self.STATE_IDLE
        self.state_start_time = time.time()

        self.detector_cw = PerpendicularDetector(self.delta_eps, self.min_stable_samples)
        self.detector_ccw = PerpendicularDetector(self.delta_eps, self.min_stable_samples)

        # --- ROS интерфейсы ---
        self.lidar_sub = self.create_subscription(
            String, 'lidar/obstacles', self.lidar_callback, 10
        )

        self.cmd_sub = self.create_subscription(
            UInt8, 'room_align_cmd', self.command_callback, 10
        )

        self.status_pub = self.create_publisher(
            UInt8, 'room_align_status', 10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.main_loop)

        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)
        self.ready_pub.publish(String(data=self.get_name()))

        self.get_logger().info("RoomAlignNode initialized (IDLE)")

    # ----------------------- callbacks -----------------------

    def lidar_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("lidar_callback: JSON decode error")
            return

        front_dist = None
        if isinstance(data, dict):
            for k, v in data.items():
                try:
                    angle = int(k)
                except (ValueError, TypeError):
                    continue
                if angle == 0:
                    front_dist = v
                    break

        if front_dist is None:
            return

        if not isinstance(front_dist, (int, float)) or math.isnan(front_dist) or math.isinf(front_dist):
            return

        if front_dist < self.min_valid_range or front_dist > self.max_valid_range * 1.5:
            return

        front_dist = min(front_dist, self.max_valid_range)

        if self.front_filtered is None:
            self.front_filtered = float(front_dist)
        else:
            self.front_filtered = (
                self.filter_alpha * float(front_dist)
                + (1.0 - self.filter_alpha) * self.front_filtered
            )

        self.front_raw = float(front_dist)
        self.last_lidar_time = time.time()

    def command_callback(self, msg: UInt8):
        if msg.data == 0:
            self.get_logger().info("Received reset command (0)")
            self.set_state(self.STATE_IDLE)
            return

        if self.state != self.STATE_IDLE:
            self.get_logger().info(f"Command {msg.data} ignored: state != IDLE")
            return

        self.get_logger().info(f"Start command received: {msg.data}")
        self.start_algorithm()

    # ----------------------- сервисные методы -----------------------

    def set_state(self, new_state: int):
        self.state = new_state
        self.state_start_time = time.time()
        self.publish_twist(0.0, 0.0)

        names = {
            self.STATE_IDLE: "IDLE",
            self.STATE_ROTATE_CW: "ROTATE_CW",
            self.STATE_FORWARD: "FORWARD",
            self.STATE_ROTATE_CCW: "ROTATE_CCW",
            self.STATE_BACKWARD: "BACKWARD",
            self.STATE_DONE: "DONE",
            self.STATE_ERROR: "ERROR",
        }
        self.get_logger().info(f"STATE: {names.get(new_state, 'UNKNOWN')}")

    def start_algorithm(self):
        if self.front_filtered is None or (time.time() - self.last_lidar_time) > self.lidar_timeout:
            self.get_logger().warn("Cannot start: no recent lidar data at 0°")
            self.set_state(self.STATE_ERROR)
            self.publish_status(2)
            return

        self.detector_cw.reset()
        self.detector_ccw.reset()
        self.set_state(self.STATE_ROTATE_CW)

    def publish_twist(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.cmd_vel_pub.publish(msg)

    def publish_status(self, code: int):
        msg = UInt8()
        msg.data = code
        self.status_pub.publish(msg)

    def lidar_is_fresh(self) -> bool:
        return (time.time() - self.last_lidar_time) <= self.lidar_timeout

    # ----------------------- главный цикл -----------------------

    def main_loop(self):
        if self.state not in [self.STATE_IDLE, self.STATE_DONE, self.STATE_ERROR]:
            if not self.lidar_is_fresh():
                self.get_logger().error("Lidar data timeout — stopping and switching to ERROR")
                self.set_state(self.STATE_ERROR)
                self.publish_status(2)
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
            # успех: отправляем 1 и возвращаемся в IDLE
            self.publish_status(1)
            self.set_state(self.STATE_IDLE)

        elif self.state == self.STATE_ERROR:
            self.publish_twist(0.0, 0.0)

    # ----------------------- этапы -----------------------

    def step_rotate_cw(self):
        if (time.time() - self.state_start_time) > self.max_rotate_time:
            self.get_logger().error("Timeout in ROTATE_CW")
            self.set_state(self.STATE_ERROR)
            self.publish_status(2)
            return

        d = self.front_filtered
        if d is None:
            self.publish_twist(0.0, -self.angular_speed)
            return

        found = self.detector_cw.update(d)

        if not found:
            self.publish_twist(0.0, -self.angular_speed)
        else:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"Perpendicular (CW) detected at distance {d:.3f} m")
            self.set_state(self.STATE_FORWARD)

    def step_forward(self):
        if (time.time() - self.state_start_time) > self.max_forward_time:
            self.get_logger().error("Timeout in FORWARD")
            self.set_state(self.STATE_ERROR)
            self.publish_status(2)
            return

        d = self.front_filtered
        if d is None:
            self.publish_twist(0.0, 0.0)
            return

        target = self.target_front_dist
        tol = self.distance_tolerance

        if d > (target + tol):
            self.publish_twist(self.linear_speed, 0.0)
        else:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"Forward reached: d_front = {d:.3f} m")
            self.detector_ccw.reset()
            self.set_state(self.STATE_ROTATE_CCW)

    def step_rotate_ccw(self):
        if (time.time() - self.state_start_time) > self.max_rotate_time:
            self.get_logger().error("Timeout in ROTATE_CCW")
            self.set_state(self.STATE_ERROR)
            self.publish_status(2)
            return

        d = self.front_filtered
        if d is None:
            self.publish_twist(0.0, self.angular_speed)
            return

        found = self.detector_ccw.update(d)

        if not found:
            self.publish_twist(0.0, self.angular_speed)
        else:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"Perpendicular (CCW) detected at distance {d:.3f} m")
            # после выравнивания — новый шаг: отъезд назад
            self.set_state(self.STATE_BACKWARD)

    def step_backward(self):
        """
        Движение назад до тех пор, пока переднее расстояние не станет >= target_back_dist.
        """
        if (time.time() - self.state_start_time) > self.max_backward_time:
            self.get_logger().error("Timeout in BACKWARD")
            self.set_state(self.STATE_ERROR)
            self.publish_status(2)
            return

        d = self.front_filtered
        if d is None:
            # лучше остановиться, чем ехать в слепую
            self.publish_twist(0.0, 0.0)
            return

        target = self.target_back_dist
        tol = self.distance_tolerance

        # хотим: d_front > 2.1 м (≈ target_back_dist)
        if d < (target - tol):
            # ещё близко — сдаём назад
            self.publish_twist(-self.linear_speed, 0.0)
        else:
            # достигли нужной дистанции
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f"Backward finished: d_front = {d:.3f} m")
            self.set_state(self.STATE_DONE)


def main(args=None):
    rclpy.init(args=args)
    node = RoomAlignNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
