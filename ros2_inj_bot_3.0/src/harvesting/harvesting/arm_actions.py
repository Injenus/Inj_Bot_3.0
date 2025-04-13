import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, UInt8
import json

class ArmActions(Node):
    def __init__(self):
        super().__init__('arm_actions')
        self.current_state = -1
        self.last_state = -1
        
        # Инициализация подписчика и публикатора
        self.subscription = self.create_subscription(String, 'arm_action', self.command_callback, 10)
        self.publisher = self.create_publisher(UInt8, 'servo/lut', 10)
        
        # Таймер для выполнения действий
        self.action_timer = self.create_timer(0.1, self.execute_action)

    def command_callback(self, msg):
        """Обработчик входящих команд"""
        try:
            command = json.loads(msg.data)
            new_state = self.parse_command(command)
            
            if new_state != self.current_state:
                self.get_logger().info(f"State changed: {self.current_state} -> {new_state}")
                self.last_state = self.current_state
                self.current_state = new_state
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid command: {e}")

    def parse_command(self, command):
        """Парсинг команд"""
        states = {
            'stop': -1, # в положение для сенса объектов
            'knock_down': 0,
            'pick': 1,
            'throw_short_side': 2,
            'throw_long_side': 3
        }
        return states.get(command, -1)

    def execute_action(self):
        """Основной цикл выполнения действий"""
        if self.current_state == -1:
            if self.last_state != -1:
                self.cleanup_action(self.last_state)
            return

        try:
            if self.current_state == 0:
                self.knock_down_action()
            elif self.current_state == 1:
                self.pick_action()
            elif self.current_state == 2:
                self.throw_short_side_action()
            elif self.current_state == 3:
                self.throw_long_side_action()
                
        except Exception as e:
            self.get_logger().error(f"Action failed: {e}")

    def publish_command(self, number):
        """Потокобезопасная публикация"""
        msg = UInt8()
        msg.data = number
        self.publisher.publish(msg)

    def knock_down_action(self):
        """Действие для сбивания"""
        self.publish_command(-2)
        self.publish_command(-1)

    def pick_action(self):
        """Действие для подбора"""
        self.publish_command(-3)
        self.publish_command(-4)
        self.publish_command(-5)
        self.publish_command(-1)

    def throw_short_side_action(self):
        """Бросок на короткую дистанцию"""
        pass

    def throw_long_side_action(self):
        """Бросок на длинную дистанцию"""
        pass

    def cleanup_action(self, previous_state):
        """Очистка ресурсов предыдущего действия"""
        self.publish_command(10)
        self.get_logger().info(f"Cleaning up after state {previous_state}")
        self.last_state = -1

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