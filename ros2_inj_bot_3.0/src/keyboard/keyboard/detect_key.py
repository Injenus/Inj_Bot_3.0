import evdev
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
import threading
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
import copy

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('detect_key')

        self.LINEAR_SPEED_X = 0.2 # м/с
        self.LINEAR_SPEED_Y = 0.2  # м/с
        self.ANGULAR_SPEED = 0.5 # рад/с

        with open('../../../../arm_n_cam_mount_api_8_bytes/config_arm.json', 'r') as file:
            self.config_arm = json.load(file)
            # l1, l2, l3 = config['length']
            # ang_range_arm = config['ang_range']

        with open('../../../../arm_n_cam_mount_api_8_bytes/config_cam.json', 'r') as file:
            self.config_cam = json.load(file)
            # l1, l2, l3 = config['length']
            # ang_range_arm = config['ang_range']

        self.SERVO_FREQ = 50
        self.SERVO_DEG_PER_SEC = 15
        self.SERVO_ANG_STEP = self.SERVO_DEG_PER_SEC / self.SERVO_FREQ
        self.SERVO_MM_PER_SEC = 40
        self.SERVO_LINEAR_STEP = self.SERVO_MM_PER_SEC / self.SERVO_FREQ

        self.SERVO_RANGE = [self.config_arm['ang_range']] + [self.config_cam['ang_range']]
        self.SERVO_OFFSET = self.config_arm['offset'] + self.config_cam['offset']
    
        self.last_servo_pos = copy.deepcopy(self.SERVO_OFFSET)

        self.current_gener_coord = copy.deepcopy(self.last_servo_pos)
        self.is_polar = True
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.servo_pub = self.create_publisher(UInt8MultiArray, 'servo/to_write', 1)
        self.servo_subs = self.create_subscription(UInt8MultiArray, 'servo/current', self.get_last_servo_pos, 1)

        self.cam_keys = {
            ecodes.KEY_UP: 'up',
            ecodes.KEY_DOWN: 'down',
            ecodes.KEY_LEFT: 'left',
            ecodes.KEY_RIGHT: 'right'
        }
        self.wheel_keys = {
            ecodes.KEY_Q: 'q',
            ecodes.KEY_W: 'w',
            ecodes.KEY_E: 'e',
            ecodes.KEY_A: 'a',
            ecodes.KEY_S: 's',
            ecodes.KEY_D: 'd'
        }
        """
        4-6 theta или x
        2-8 h или z
        1-7 r или y
        3-9 захват
        0-5 цилиндрические или декартовые 
        """
        self.arm_keys = {
            ecodes.KEY_KP0: '0',
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

        self.device = self.find_keyboard_device()
        if not self.device:
            self.get_logger().error('Keyboard not found. Exiting.')
            raise RuntimeError("Keyboard device not found")
        
        self.get_logger().info(f"Using device: {self.device.name}")
        
        # Состояние клавиш и блокировка для потокобезопасности
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        
        # Поток для чтения событий клавиатуры
        self.read_thread = threading.Thread(target=self.read_events)
        self.read_thread.start()
        
        # Таймер для обработки нажатий
        self.timer = self.create_timer(1/self.SERVO_FREQ, self.process_keys)

    def get_last_servo_pos(self, msg):
        self.last_servo_pos = [x - y for x, y in zip(list(msg.data), self.SERVO_OFFSET)] # приводим к отрицательному виду (полному виду, реальный дипазон грудсов)
        self.current_gener_coord = copy.deepcopy(self.last_servo_pos)

    def find_keyboard_device(self):
        """Находит устройство клавиатуры, поддерживающее нужные клавиши."""
        # Объединяем все клавиши из трех групп
        self.all_keys = {
            **self.wheel_keys,
            **self.cam_keys,
            **self.arm_keys
        }
        devices = [InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            try:
                caps = device.capabilities()
                if ecodes.EV_KEY in caps:
                    available_keys = caps[ecodes.EV_KEY]
                    # Проверяем наличие хотя бы одной нужной клавиши
                    if any(key in self.all_keys for key in available_keys):
                        return device
            except (OSError, PermissionError):
                continue
        return None

    def read_events(self):
        """Читает события клавиатуры в отдельном потоке."""
        try:
            for event in self.device.read_loop():
                if event.type == ecodes.EV_KEY:
                    with self.lock:
                        key_code = event.code
                        key_name = None
                        
                        # Проверяем все три группы клавиш
                        if key_code in self.wheel_keys:
                            key_name = self.wheel_keys[key_code]
                        elif key_code in self.cam_keys:
                            key_name = self.cam_keys[key_code]
                        elif key_code in self.arm_keys:
                            key_name = self.arm_keys[key_code]

                        if key_name is not None:
                            if event.value == 1:  # Нажатие
                                self.pressed_keys.add(key_name)
                            elif event.value == 0:  # Отпускание
                                self.pressed_keys.discard(key_name)
        except Exception as e:
            self.get_logger().error(f"Error reading events: {str(e)}")
            self.stop_event.set()
    
    def process_keys(self):
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
        
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        
        # Движение по X (W/S)
        if 'w' in current_keys and 's' not in current_keys:
            linear_x = self.LINEAR_SPEED_X
        elif 's' in current_keys and 'w' not in current_keys:
            linear_x = -self.LINEAR_SPEED_X
        
        # Движение по Y (A/D)
        if 'a' in current_keys and 'd' not in current_keys:
            linear_y = self.LINEAR_SPEED_Y
        elif 'd' in current_keys and 'a' not in current_keys:
            linear_y = -self.LINEAR_SPEED_Y
        
        # Вращение по Z (Q/E)
        if 'q' in current_keys and 'e' not in current_keys:
            angular_z = self.ANGULAR_SPEED
        elif 'e' in current_keys and 'q' not in current_keys:
            angular_z = -self.ANGULAR_SPEED
        #-----------------------------------------------------
        # переключение типа координат
        if '0' in current_keys and '5' not in current_keys:
            self.is_polar = True
        elif '5' in current_keys and '5' not in current_keys:
            self.is_polar = False

        if self.is_polar:
            # вращение базы манипулятора
            if '4' in current_keys and '6' not in current_keys:
                self.current_gener_coord[0] -= self.SERVO_ANG_STEP
            elif '6' in current_keys and '4' not in current_keys:
                self.current_gener_coord[0] += self.SERVO_ANG_STEP

        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        
        self.twist_pub.publish(twist_msg)
        self.get_logger().debug(
            f"PUB: lin_x={linear_x}, lin_y={linear_y}, ang_z={angular_z}",
            throttle_duration_sec=0.2
        )
        for i, el in enumerate(self.current_gener_coord):
            self.current_gener_coord[i] = round(el) + self.SERVO_OFFSET[i] # приводи к положиетльному виду

        servo_msg = UInt8MultiArray()
        servo_msg.data = bytes(self.last_servo_pos)
        self.servo_pub.publish(servo_msg)
        self.get_logger().debug(
            f"PUB: {servo_msg.data}",
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