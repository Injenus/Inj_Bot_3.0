import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, UInt8
import json
import time
import threading
import queue

import sys, os
modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf

class ArmActionExecutor:
    """Класс для последовательного выполнения действий в фоновом потоке"""
    def __init__(self, publish_callback):
        self.publish = publish_callback
        self.task_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.worker_thread = threading.Thread(target=self._run)
        self.worker_thread.start()

    def _run(self):
        """Основной цикл обработки задач"""
        while not self.stop_event.is_set():
            try:
                action = self.task_queue.get(timeout=conf.dt_arm_action)
                action()
            except queue.Empty:
                continue

    def add_action(self, action_func):
        """Добавление действия в очередь"""
        self.task_queue.put(action_func)

    def shutdown(self):
        """Корректное завершение потока"""
        self.stop_event.set()
        self.worker_thread.join()

class ArmActions(Node):
    def __init__(self):
        super().__init__('arm_actions')

        self.executor = ArmActionExecutor(self.publish_command)

        self.current_state = -1
        
        self.subscription = self.create_subscription(String, 'arm_action', self.command_callback, 10)
        self.publisher = self.create_publisher(UInt8, 'servo/lut', 10)
        
        self.action_timer = self.create_timer(conf.dt_arm_action, self.execute_action)

        self.state_lock = threading.Lock()

    def command_callback(self, msg):
        """Асинхронная обработка входящих команд"""
        try:
            command = json.loads(msg.data)
            new_state = conf.arm_states_table.get(command, -1)

            with self.state_lock:
                if new_state != self.current_state:
                    self.get_logger().info(f"State changed: {new_state}")
                    self.current_state = new_state
                    self.executor.add_action(lambda: self.execute_action_sequence(new_state))

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid command: {e}")


    # def execute_action(self):
    #     """Основной цикл выполнения действий"""
    #     with self.state_lock:
    #         try:
    #             if self.current_state == -2:
    #                 self.safe_init()
    #             if self.current_state == -1:
    #                 self.init()
    #             elif self.current_state == 0:
    #                 self.knock_down_action()
    #             elif self.current_state == 1:
    #                 self.pick_action()
    #             elif self.current_state == 2:
    #                 self.throw_short_side_action()
    #             elif self.current_state == 3:
    #                 self.throw_long_side_action()
                    
    #         except Exception as e:
    #             self.get_logger().error(f"Action failed: {e}")

    def execute_action_sequence(self, state):
        """Выполнение последовательности действий для состояния"""
        actions = self.get_actions_for_state(state)
        for action in actions:
            self.publish_command(action['command'])
            time.sleep(action['delay'])  # Блокирующий sleep только в рабочем потоке

    def get_actions_for_state(self, state):
        """Определение последовательности действий (как в вашем оригинальном коде)"""
        return {
            -2: [{'safe_init': 10, 'delay': conf.delay_init_safe}],
            -1: [{'init_(search_fruit)': 11, 'delay': conf.delay_init}],
            0: [
                {'knock_down': 12, 'delay': conf.delay_1_knock},
                {'init': 11, 'delay': conf.delay_init}
            ],
            1: [
                {'down': 13, 'delay': conf.delay_1_pick},
                {'lengthing': 14, 'delay': conf.delay_2_pick},
                {'up': 15, 'delay': conf.delay_3_pick},
                {'turn': 18, 'delay': conf.delay_4_pick},
                {'init': 11, 'delay': conf.delay_init}
            ],
            2: [
                {'short_throw': 17, 'delay': conf.delay_throw_short},
                {'init': 11, 'delay': conf.delay_init}
            ],
            3: [
                {'long_throw': 16, 'delay': conf.delay_throw_long},
                {'init': 11, 'delay': conf.delay_init}
            ],
            4: [
                {'safe_turn_for_direct': 18, 'delay': 0.4},
                {'direct_folded': 5, 'delay': 0.3},
                {'init': 11, 'delay': conf.delay_init}
            ]
        }.get(state, [])

    def publish_command(self, number):
        """Потокобезопасная публикация"""
        msg = UInt8()
        msg.data = number
        self.publisher.publish(msg)

    # def safe_init(self):
    #     """Поворот вбок перед всем, чтобы не ёбнуть периферию"""
    #     self.publish_command(10)
    #     time.sleep(0.8)

    # def init(self):
    #     """Возврат в положение для сёрча"""
    #     self.publish_command(11)
    #     time.sleep(0.2)

    # def knock_down_action(self):
    #     """Действие для сбивания"""
    #     self.publish_command(12)
    #     time.sleep(0.5)
    #     self.publish_command(11)
    #     time.sleep(0.5)

    # def pick_action(self):
    #     """Действие для подбора"""
    #     self.publish_command(13)
    #     time.sleep(1.)
    #     self.publish_command(14)
    #     time.sleep(1.)
    #     self.publish_command(15)
    #     time.sleep(1.)
    #     self.publish_command(18)
    #     time.sleep(1.)
    #     self.publish_command(11)
    #     time.sleep(0.2)

    # def throw_short_side_action(self):
    #     """Бросок на короткую дистанцию"""
    #     self.publish_command(16)
    #     time.sleep(1.0)
    #     self.publish_command(11)
    #     time.sleep(2.0)

    # def throw_long_side_action(self):
    #     """Бросок на длинную дистанцию"""
    #     self.publish_command(17)
    #     time.sleep(1.0)
    #     self.publish_command(11)
    #     time.sleep(2.0)

    def destroy_node(self):
        self.executor.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmActions()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()