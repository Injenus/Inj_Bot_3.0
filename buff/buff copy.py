import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Event, Lock
from std_msgs.msg import String

class ManagedThread:
    def __init__(self, name, node, parent=None):
        self.name = name
        self.node = node
        self.parent = parent
        self.thread = None
        self.running = False
        self.paused = Event()
        self.child = None
        self.lock = Lock()
        
        # ROS2 компоненты (пример)
        self.publisher = self.node.create_publisher(String, f'{name}_topic', 10)

    def start(self):
        with self.lock:
            if not self.running:
                self.running = True
                self.thread = Thread(target=self.run, daemon=True)
                self.thread.start()
                self.node.get_logger().info(f"{self.name} started")

    def run(self):
        while self.running and rclpy.ok():
            if self.paused.is_set():
                self.paused.wait()
            
            # Основная логика выполнения
            self.execute_logic()
            
            # Проверка условия для запуска child
            if self.should_start_child():
                self.start_child()

    def execute_logic(self):
        """Основная рабочая логика потока (переопределить)"""
        msg = String()
        msg.data = f"Activity from {self.name}"
        self.publisher.publish(msg)
        time.sleep(0.5)

    def should_start_child(self):
        """Условие для запуска дочернего потока (переопределить)"""
        return False

    def start_child(self):
        with self.lock:
            if not self.child:
                self.paused.set()
                self.child = ManagedThread(
                    name=f"{self.name}_child",
                    node=self.node,
                    parent=self
                )
                self.child.start()
                self.node.get_logger().info(f"{self.name} paused, child started")

    def stop(self):
        with self.lock:
            self.running = False
            self.paused.clear()
            if self.child:
                self.child.stop()
            if self.parent:
                self.parent.resume()
            self.node.get_logger().info(f"{self.name} stopped")

    def resume(self):
        with self.lock:
            self.paused.clear()
            self.node.get_logger().info(f"{self.name} resumed")

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        
        # Инициализация всех потоков
        self.main_thread = ManagedThread("main", self)
        self.child_threads = [
            ManagedThread(f"child_{i}", self, parent=self.main_thread)
            for i in range(6)
        ]
        self.grandchild_threads = [
            ManagedThread(f"grandchild_{i}", self, parent=self.child_threads[i])
            for i in range(6)
        ]
        
        # ROS2 подписка
        self.subscription = self.create_subscription(
            String,
            'trigger',
            self.trigger_callback,
            10)
        
        self.condition_lock = Lock()
        self.trigger_conditions = [False]*6

    def trigger_callback(self, msg):
        with self.condition_lock:
            # Логика обработки триггеров
            for i in range(6):
                if msg.data == f"start_child_{i}":
                    self.trigger_conditions[i] = True

    def start_system(self):
        self.main_thread.start()
        
    def stop_system(self):
        self.main_thread.stop()
        for t in self.child_threads + self.grandchild_threads:
            t.stop()

def main(args=None):
    rclpy.init(args=args)
    coordinator = Coordinator()
    
    try:
        coordinator.start_system()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(coordinator)
        executor.spin()
    finally:
        coordinator.stop_system()
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()