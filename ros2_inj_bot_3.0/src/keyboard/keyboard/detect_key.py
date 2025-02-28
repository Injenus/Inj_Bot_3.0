import evdev
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Twist

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('detect_key')

        # Добавляем константы скоростей
        self.LINEAR_SPEED_X = 0.2 # м/с
        self.LINEAR_SPEED_Y = 0.2  # м/с
        self.ANGULAR_SPEED = 0.5 # рад/с
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.keys_of_interest = {
            ecodes.KEY_Q: 'q',
            ecodes.KEY_W: 'w',
            ecodes.KEY_E: 'e',
            ecodes.KEY_A: 'a',
            ecodes.KEY_S: 's',
            ecodes.KEY_D: 'd',
            ecodes.KEY_KP1: '1',
            ecodes.KEY_KP2: '2',
            ecodes.KEY_KP3: '3',
            ecodes.KEY_KP4: '4',
            ecodes.KEY_KP5: '5',
            ecodes.KEY_KP6: '6',
            ecodes.KEY_KP7: '7',
            ecodes.KEY_KP8: '8',
            ecodes.KEY_KP9: '9',
            ecodes.KEY_ESC: 'Esc'
        }
        
        self.device = self._find_keyboard_device()
        if not self.device:
            self.get_logger().error('Keyboard not found. Exiting.')
            raise RuntimeError("Keyboard device not found")
        
        self.get_logger().info(f"Using device: {self.device.name}")
        
        # Состояние клавиш и блокировка для потокобезопасности
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        
        # Поток для чтения событий клавиатуры
        self.read_thread = threading.Thread(target=self._read_events)
        self.read_thread.start()
        
        # Таймер для обработки нажатий
        self.timer = self.create_timer(0.005, self._process_keys)  # 200 Hz

    def _find_keyboard_device(self):
        """Находит устройство клавиатуры, поддерживающее нужные клавиши."""
        devices = [InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            try:
                caps = device.capabilities()
                if ecodes.EV_KEY in caps:
                    available_keys = caps[ecodes.EV_KEY]
                    if any(key in available_keys for key in self.keys_of_interest):
                        return device
            except (OSError, PermissionError):
                continue
        return None

    def _read_events(self):
        """Читает события клавиатуры в отдельном потоке."""
        try:
            for event in self.device.read_loop():
                if event.type == ecodes.EV_KEY:
                    with self.lock:
                        key_code = event.code
                        if key_code in self.keys_of_interest:
                            key_name = self.keys_of_interest[key_code]
                            if event.value == 1:  # Нажатие
                                self.pressed_keys.add(key_name)
                            elif event.value == 0:  # Отпускание
                                self.pressed_keys.discard(key_name)
        except Exception as e:
            self.get_logger().error(f"Error reading events: {str(e)}")
            self.stop_event.set()
    
    def _process_keys(self):
        """Обрабатывает текущее состояние клавиш."""
        if self.stop_event.is_set():
            return
        
        # Копируем состояние для минимизации времени блокировки
        with self.lock:
            current_keys = self.pressed_keys.copy()
        
        # Завершение работы по Esc
        if 'Esc' in current_keys:
            self.get_logger().info("Escape pressed. Shutting down.")
            self.stop_event.set()
            self.destroy_node()
            rclpy.shutdown()
            return
        
        # Инициализация скоростей
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        
        # Обработка движения по X (W/S)
        if 'w' in current_keys and 's' not in current_keys:
            linear_x = self.LINEAR_SPEED_x
        elif 's' in current_keys and 'w' not in current_keys:
            linear_x = -self.LINEAR_SPEED_x
        
        # Обработка движения по Y (A/D)
        if 'a' in current_keys and 'd' not in current_keys:
            linear_y = self.LINEAR_SPEED_Y
        elif 'd' in current_keys and 'a' not in current_keys:
            linear_y = -self.LINEAR_SPEED_Y
        
        # Обработка вращения по Z (Q/E)
        if 'q' in current_keys and 'e' not in current_keys:
            angular_z = self.ANGULAR_SPEED
        elif 'e' in current_keys and 'q' not in current_keys:
            angular_z = -self.ANGULAR_SPEED

        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        
        self.twist_pub.publish(twist_msg)
        
        self.get_logger().debug(
            f"PUB: lin_x={linear_x}, lin_y={linear_y}, ang_z={angular_z}",
            throttle_duration_sec=0.2
        )

    def destroy_node(self):
        self.stop_event.set()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detect_key = KeyboardNode()
    try:
        rclpy.spin(detect_key)
    except:
        detect_key.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()