import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8
import json
import threading
import asyncio

class ArmActions(Node):
    def __init__(self):
        super().__init__('arm_actions')
        self.state = -1
        self.stop_event = threading.Event()
        self.task_thread = None
        
        self.subscription = self.create_subscription(String, 'arm_action', self.command_callback, 3)
        self.publisher = self.create_publisher(UInt8, 'servo/lut', 10)
        
        self.timer = self.create_timer(0.5, self.state_check)

    def command_callback(self, msg):
        try:
            command = json.loads(msg.data)
            new_state = self.parse_command(command)
            if new_state != self.state:
                self.get_logger().info(f"State changed: {self.state} -> {new_state}")
                self.state = new_state
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid command: {e}")

    def parse_command(self, command):
        """Парсинг команд с защитой от ошибок"""
        return {
            'stop': -1,
            'knock_down': 0,
            'pick': 1,
            'throw_short_side': 2,
            'throw_long_side': 3
        }.get(command, -1)

    async def async_action(self, state):
        """Основная асинхронная задача"""
        try:
            while not self.stop_event.is_set() and self.state == state:
                await asyncio.sleep(0.01)
                self.execute_action(state)
        except Exception as e:
            #self.get_logger().error(f"Action failed: {e}")
            pass
        finally:
            #self.get_logger().info(f"Action {state} completed")
            pass

    def execute_action(self, state):
        """Логика выполнения действий"""
        #self.get_logger().info(f"Executing action for state: {state}")
        def publ(number):
            msg = UInt8()
            msg.data = number
            self.publisher.publish(msg)
            await asyncio.sleep(0.5)

        if state == 0:
            publ(-2)
            publ(-1)

        elif state == 1:
            publ(-3)
            publ(-4)
            publ(-5)
            publ(-1)

        elif state == 2:
            pass
        elif state == 3:
            pass
        
    def state_check(self):
        """Проверка состояния"""
        if self.state == -1:
            if self.task_thread and self.task_thread.is_alive():
                # self.get_logger().info("Stopping current action")
                self.stop_event.set()
        else:
            if not self.task_thread or not self.task_thread.is_alive():
                self.stop_event.clear()
                self.task_thread = threading.Thread(
                    target=self.run_async_action,
                    args=(self.state,)
                )
                self.task_thread.start()

    def run_async_action(self, state):
        """Запуск асинхронной задачи в потоке"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.async_action(state))
        loop.close()

    def destroy_node(self):
        self.stop_event.set()
        if self.task_thread:
            self.task_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmActions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()