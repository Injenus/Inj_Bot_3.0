import evdev
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt8


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('detect_key')

        self.arm_keys = {
            ecodes.KEY_0: '0',
            ecodes.KEY_1: '1',
            ecodes.KEY_2: '2',
            ecodes.KEY_3: '3',
            ecodes.KEY_4: '4',
            ecodes.KEY_5: '5',
            ecodes.KEY_6: '6',
            ecodes.KEY_7: '7',
            ecodes.KEY_8: '8',
            ecodes.KEY_9: '9',
            ecodes.KEY_ESC: 'Esc',
            ecodes.KEY_D: 'd',
            ecodes.KEY_T: 't',
            ecodes.KEY_H: 'h',
            ecodes.KEY_U: 'u',
            ecodes.KEY_F: 'f',
            ecodes.KEY_X: 'x',
            ecodes.KEY_S: 's',
            ecodes.KEY_E: 'e',
            ecodes.KEY_N: 'n'
        }

        self.letter_convert = {
            'd': 10,
            't': 20,
            'h': 30,
            'u': 40,
            'f': 50,
            'x': 60,
            's': 70,
            'e': 80,
            'n': 90
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
        self.timer = self.create_timer(1/200, self.process_keys)

        #self.servo_pub = self.create_publisher(UInt8MultiArray, 'servo/to_write', 1)
        self.publ_table_pos = self.create_publisher(UInt8, 'servo/lut', 3)


    def find_keyboard_device(self):
        """Поиск и выбор клавиатуры с подробной отладкой"""
        devices = [InputDevice(path) for path in evdev.list_devices()]

        self.get_logger().info("\nДоступные устройства ввода:")
        for i, dev in enumerate(devices):
            print(f"{i+1}. {dev.name} ({dev.path})")

        for dev in devices:
            try:
                caps = dev.capabilities()
                if ecodes.EV_KEY in caps:
                    # if DEBUG:
                    #     print(f"\nПроверка устройства: {dev.name}")
                    #     print("Поддерживаемые клавиши:", [ecodes.KEY[k] for k in caps[ecodes.EV_KEY]] if caps[ecodes.EV_KEY] else "Нет клавиш")
                    
                    # Проверяем наличие необходимых клавиш
                    required_keys = {ecodes.KEY_A, ecodes.KEY_P, ecodes.KEY_S}
                    if required_keys.issubset(caps[ecodes.EV_KEY]):
                        # if DEBUG:
                        #     print(f"Выбрана клавиатура: {dev.name}")
                        return dev
            except Exception as e:
                # if DEBUG:
                #     print(f"Ошибка при проверке {dev.path}: {str(e)}")
                continue

        return None


    def read_events(self):
        """Читает события клавиатуры в отдельном потоке."""
        try:
            print("TRYE Read events")
            for event in self.device.read_loop():
                if event.type == ecodes.EV_KEY:
                    with self.lock:
                        key_code = event.code
                        key_name = None
                        
                        # Проверяем все три группы клавиш
                        if key_code in self.arm_keys:
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
        
        data = None
        add = 0

        for key in current_keys:
            if key in self.letter_convert:
                add = self.letter_convert[key]
                break

        for key in current_keys:
            try:
                key = int(key)
            except:
                pass
                # if key in self.letter_convert:
                #     add = self.letter_convert[key]
            if isinstance(key, int):
                data = key + add
                self.get_logger().info(f'Command: {data}')
                break

        if data is not None:
            lut_msg = UInt8()
            lut_msg.data = data
            self.publ_table_pos.publish(lut_msg)
            print(data)


    def destroy_node(self):
        self.stop_event.set()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detect_key = KeyboardNode()
    rclpy.spin(detect_key)
    detect_key.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()