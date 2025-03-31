import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

class WheelTestNode(Node):
    def __init__(self):
        super().__init__('wheel_test_node')
        
        self.declare_parameter('test_rpm', 42)
        self.test_rpm = self.get_parameter('test_rpm').value
        
        self.wheel_pub = self.create_publisher(
            Int32MultiArray,
            'wheel/target_rpm',
            10
        )
        
        self.wheel_order = [0, 1, 2, 3]  # Порядок тестирования колёс [R1, L1, L2, R2]
        self.current_wheel = 0
        self.phase = 1  # 1 - первая фаза (5 сек), -1 - вторая фаза
        self.start_phase_time = time.time()
        self.start_wheel_time = time.time()
        
        self.timer = self.create_timer(0.005, self.timer_callback)
        self.get_logger().info("Начало теста колёс")

    def timer_callback(self):
        current_time = time.time()
        elapsed_phase = current_time - self.start_phase_time
        elapsed_wheel = current_time - self.start_wheel_time

        # Завершение работы после теста всех колёс
        if self.current_wheel >= len(self.wheel_order):
            self.get_logger().info("Тест завершён")
            while(1):
                msg = Int32MultiArray()
                msg.data = [0,0,0,0]
                self.wheel_pub.publish(msg)
                time.sleep(1)

        # Переключение на следующее колесо через 10 секунд
        if elapsed_wheel >= 10:
            self.current_wheel += 1
            if self.current_wheel < len(self.wheel_order):
                self.start_wheel_time = current_time
                self.start_phase_time = current_time
                self.phase = 1
                self.get_logger().info(f"Переход к колесу {self.current_wheel}")
            return

        # Смена направления через каждые 5 секунд
        if elapsed_phase >= 5:
            self.phase *= -1
            self.start_phase_time = current_time
            self.get_logger().info(f"Смена направления колеса {self.current_wheel}")

        # Создание массива RPM
        rpm = [0, 0, 0, 0]
        if self.current_wheel < len(self.wheel_order):
            target_wheel = self.wheel_order[self.current_wheel]
            rpm[target_wheel] = self.test_rpm * self.phase

        # Публикация команд
        msg = Int32MultiArray()
        msg.data = rpm
        self.wheel_pub.publish(msg)

    def destroy_node(self):
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WheelTestNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info("Узел остановлен")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()