from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Правильный профиль для высокочастотных данных
qos_profile = QoSProfile(
    depth=100,  # Глубина очереди (минимум 10-100 для high-frequency)
    reliability=QoSReliabilityPolicy.RELIABLE,  # Обязательно для синхронизации с rosbag
    durability=QoSDurabilityPolicy.VOLATILE  # Совместимость с записью по умолчанию
)

self.subscription = self.create_subscription(
    Int32MultiArray,
    '/wheel/target_rpm',
    self.listener_callback,
    qos_profile=qos_profile  # Явное указание профиля
)