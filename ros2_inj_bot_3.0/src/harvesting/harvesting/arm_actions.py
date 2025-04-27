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

        self.arm_act_executor = ArmActionExecutor(self.publish_command)

        self.current_state = -3
        
        self.subscription = self.create_subscription(String, 'arm_action', self.command_callback, 10)
        self.publisher = self.create_publisher(UInt8, 'servo/lut', 10)
        
        #self.action_timer = self.create_timer(conf.dt_arm_action, self.execute_action)

        self.state_lock = threading.Lock()

        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)
        msg = String(data = self.get_name())
        self.ready_pub.publish(msg)

    def command_callback(self, msg):
        """Асинхронная обработка входящих команд"""
        command = msg.data
        new_state = conf.arm_states_table.get(command, -1)
        print(new_state)

        with self.state_lock:
            if new_state != self.current_state:
                self.get_logger().info(f"State changed: {new_state}")
                self.current_state = new_state
                self.arm_act_executor.add_action(lambda: self.execute_action(new_state))
                print('y')


    def execute_action(self, state):
        """Выполнение последовательности действий для состояния"""
        actions = self.get_actions_for_state(state)
        for action in actions:
            self.publish_command(action['command'])  # Теперь берем правильный ключ
            self.get_logger().info(f"Executing: {action['name']} ({action['command']})")
            time.sleep(action['delay'])
            #time.sleep(5.0)

    def get_actions_for_state(self, state):
        """Словарь константных команд с правильными ключами"""
        return {
            -2: [
                {'command': 10, 'name': 'search_fruit_pre', 'delay': conf.st_delay},
            ],
            -1: [
                {'command': 63, 'name': '_', 'delay': conf.st_delay},
                {'command': 60, 'name': '_', 'delay': conf.st_delay},

                # {'command': 23, 'name': 'pre_knock_down_fruit', 'delay': 3*conf.st_delay},
                # {'command': 11, 'name': 'search_fruit', 'delay': conf.st_delay}
            ],
            0: [
                {'command': 63, 'name': '_', 'delay': conf.st_delay},
                {'command': 10, 'name': 'search_fruit_pre', 'delay': conf.st_delay},

                {'command': 23, 'name': 'pre_knock_down_fruit', 'delay': 2*conf.st_delay},
                {'command': 12, 'name': 'knock_down_fruit', 'delay': 2*conf.st_delay},
                {'command': 24, 'name': 'after_knock_down_fruit', 'delay': 2*conf.st_delay},
                {'command': 12, 'name': 'knock_down_fruit', 'delay': 2*conf.st_delay},
                {'command': 23, 'name': 'pre_knock_down_fruit', 'delay': 2*conf.st_delay},

                {'command': 63, 'name': 'search_fruit', 'delay': conf.st_delay},
                {'command': 60, 'name': '_', 'delay': conf.st_delay},
            ],
            1: [
                {'command': 63, 'name': '_', 'delay': conf.st_delay},
                {'command': 10, 'name': 'search_fruit_pre', 'delay': conf.st_delay},
                {'command': 23, 'name': 'pre_knock_down_fruit', 'delay': conf.st_delay},
                {'command': 11, 'name': 'search_fruit', 'delay': conf.st_delay},
                {'command': 13, 'name': '_', 'delay': conf.st_delay},
                {'command': 25, 'name': '_', 'delay': conf.st_delay},
                {'command': 26, 'name': '_', 'delay': conf.st_delay},
                {'command': 27, 'name': '_', 'delay': conf.st_delay},
                {'command': 28, 'name': '_', 'delay': conf.st_delay},
                {'command': 29, 'name': '_', 'delay': conf.st_delay},
                {'command': 30, 'name': '_', 'delay': conf.st_delay},
                {'command': 31, 'name': '_', 'delay': conf.st_delay},
                {'command': 32, 'name': '_', 'delay': conf.st_delay},
                {'command': 33, 'name': '_', 'delay': conf.st_delay},
                {'command': 15, 'name': '_', 'delay': conf.st_delay},
                {'command': 34, 'name': '_', 'delay': conf.st_delay},
                {'command': 38, 'name': '_', 'delay': conf.st_delay},
                {'command': 35, 'name': '_', 'delay': conf.st_delay},
                {'command': 36, 'name': '_', 'delay': conf.st_delay},                
            ],
            2: [
                {'command': 17, 'name': '_', 'delay': conf.st_delay},
                {'command': 37, 'name': '_', 'delay': conf.st_delay},

                {'command': 39, 'name': '_', 'delay': conf.st_delay},
                {'command': 40, 'name': '_', 'delay': conf.st_delay},
                {'command': 41, 'name': '_', 'delay': conf.st_delay},
                {'command': 42, 'name': '_', 'delay': conf.st_delay},
                {'command': 43, 'name': '_', 'delay': conf.st_delay},
                {'command': 44, 'name': '_', 'delay': conf.st_delay},
                {'command': 45, 'name': '_', 'delay': conf.st_delay},
                {'command': 46, 'name': '_', 'delay': conf.st_delay},
                {'command': 47, 'name': '_', 'delay': conf.st_delay},
                {'command': 48, 'name': '_', 'delay': conf.st_delay},
                {'command': 49, 'name': '_', 'delay': conf.st_delay},
                {'command': 50, 'name': '_', 'delay': conf.st_delay},
                {'command': 51, 'name': '_', 'delay': conf.st_delay},
                {'command': 52, 'name': '_', 'delay': conf.st_delay},
                {'command': 53, 'name': '_', 'delay': conf.st_delay},
                {'command': 54, 'name': '_', 'delay': conf.st_delay},
                {'command': 55, 'name': '_', 'delay': conf.st_delay},
                {'command': 56, 'name': '_', 'delay': conf.st_delay},

                {'command': 50, 'name': '_', 'delay': conf.st_delay},
                {'command': 47, 'name': '_', 'delay': conf.st_delay},
                {'command': 43, 'name': '_', 'delay': conf.st_delay},
                {'command': 39, 'name': '_', 'delay': conf.st_delay},
                {'command': 37, 'name': '_', 'delay': conf.st_delay},

                {'command': 17, 'name': '_', 'delay': conf.st_delay},
                {'command': 10, 'name': 'search_fruit_pre', 'delay': 2*conf.st_delay},

                {'command': 63, 'name': 'pre_knock_down_fruit', 'delay': conf.st_delay},
                {'command': 60, 'name': 'search_fruit', 'delay': conf.st_delay},
            ],
            3: [
                # from 60 to 11
                {'command': 63, 'name': '_', 'delay': conf.st_delay},
                {'command': 10, 'name': 'search_fruit_pre', 'delay': conf.st_delay},
                {'command': 23, 'name': 'pre_knock_down_fruit', 'delay': 2*conf.st_delay},
                {'command': 11, 'name': 'search_fruit', 'delay': conf.st_delay},
            ],
            4: [
                # from 11 to 60
                {'command': 23, 'name': '_', 'delay': conf.st_delay},
                {'command': 10, 'name': 'search_fruit_pre', 'delay': conf.st_delay},
                {'command': 63, 'name': 'pre_knock_down_fruit', 'delay': conf.st_delay},
                {'command': 60, 'name': 'search_fruit', 'delay': conf.st_delay},
            ]


        }.get(state, [])

    def publish_command(self, number):
        """Потокобезопасная публикация"""
        msg = UInt8()
        msg.data = number
        print('pppuuuublbll',msg.data)
        self.publisher.publish(msg)


    def destroy_node(self):
        self.arm_act_executor.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmActions()
    arm_executor = MultiThreadedExecutor()
    arm_executor.add_node(node)
    arm_executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()