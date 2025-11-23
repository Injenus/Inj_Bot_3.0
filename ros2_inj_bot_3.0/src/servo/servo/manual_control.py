import evdev
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import UInt8
import glob
import errno
import time
from collections import deque


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('detect_key_manual_control')

        # --- раскладка клавиш и кодов команд ---
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

        # буквы → десятки
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

        # текущее множество нажатых (нужно в основном для Esc)
        self.pressed_keys = set()
        self.lock = threading.Lock()

        # очередь уже сформированных команд
        self.command_queue = deque()
        self.queue_lock = threading.Lock()

        # текущая "десятка" (последняя нажатая буква)
        self.current_add = 0

        self.stop_event = threading.Event()
        self.device = None

        # паблишер
        self.publ_table_pos = self.create_publisher(UInt8, 'servo/lut', 3)

        # поток чтения клавиатуры
        self.read_thread = threading.Thread(target=self.read_events, daemon=True)
        self.read_thread.start()

        # таймер обработки очереди команд
        self.timer = self.create_timer(1 / 200.0, self.process_keys)

    # -------------------- поиск клавиатуры --------------------

    def find_keyboard_device(self):
        """
        Поиск и выбор клавиатуры.
        Сначала пытаемся /dev/input/by-id/*Keyboard*event-kbd,
        затем перебираем eventX.
        """
        # 1. Стабильные by-id пути
        by_id_paths = glob.glob('/dev/input/by-id/*Keyboard*event-kbd')
        if by_id_paths:
            for path in by_id_paths:
                try:
                    dev = InputDevice(path)
                    self.get_logger().info(
                        f"Выбрана клавиатура по by-id: {dev.name} ({dev.path})"
                    )
                    return dev
                except Exception as e:
                    self.get_logger().error(f"Не удалось открыть {path}: {e}")

        # 2. Перебор всех устройств ввода
        devices = [InputDevice(path) for path in evdev.list_devices()]

        self.get_logger().info("\nДоступные устройства ввода:")
        for i, dev in enumerate(devices):
            print(f"{i + 1}. {dev.name} ({dev.path})")

        required_keys = {
            ecodes.KEY_0, ecodes.KEY_1, ecodes.KEY_2, ecodes.KEY_3, ecodes.KEY_4,
            ecodes.KEY_5, ecodes.KEY_6, ecodes.KEY_7, ecodes.KEY_8, ecodes.KEY_9,
            ecodes.KEY_ESC
        }

        for dev in devices:
            try:
                caps = dev.capabilities()
                if ecodes.EV_KEY in caps:
                    keys = set(caps[ecodes.EV_KEY])
                    if required_keys.issubset(keys):
                        self.get_logger().info(
                            f"Выбрана клавиатура по capabilities: {dev.name} ({dev.path})"
                        )
                        return dev
            except Exception:
                continue

        return None

    # -------------------- обработка одного события клавиши --------------------

    def handle_key_event(self, key_name: str, value: int):
        """
        Обработка логики команд по событию KEY_DOWN.
        value == 1  → нажатие
        value == 0  → отпускание
        """
        # поддерживаем pressed_keys для Esc и общей диагностики
        if value == 1:
            self.pressed_keys.add(key_name)
        elif value == 0:
            self.pressed_keys.discard(key_name)

        # дальше нас интересуют только нажатия
        if value != 1:
            return

        # Esc обрабатывается в process_keys как "мягкое завершение"
        if key_name == 'Esc':
            return

        # буква → десятки
        if key_name in self.letter_convert:
            self.current_add = self.letter_convert[key_name]
            return

        # цифра → команда
        try:
            digit = int(key_name)
        except ValueError:
            return

        cmd = digit + self.current_add
        with self.queue_lock:
            self.command_queue.append(cmd)
        self.get_logger().info(f'Queued command: {cmd}')

    # -------------------- чтение событий с авто-reconnect --------------------

    def read_events(self):
        """Читает события клавиатуры в отдельном потоке с авто-reconnect."""
        self.get_logger().info("Старт потока чтения клавиатуры")
        backoff = 1.0

        while not self.stop_event.is_set():
            # если устройства нет — пытаемся найти
            if self.device is None:
                self.device = self.find_keyboard_device()
                if self.device is None:
                    self.get_logger().warn(
                        f"Клавиатура не найдена, повтор через {backoff:.1f} сек"
                    )
                    time.sleep(backoff)
                    backoff = min(backoff * 2.0, 10.0)
                    continue
                backoff = 1.0

            try:
                for event in self.device.read_loop():
                    if self.stop_event.is_set():
                        break

                    if event.type == ecodes.EV_KEY:
                        key_code = event.code
                        if key_code in self.arm_keys:
                            key_name = self.arm_keys[key_code]
                            with self.lock:
                                self.handle_key_event(key_name, event.value)

            except OSError as e:
                if e.errno == errno.ENODEV:
                    self.get_logger().error(
                        f"Устройство ввода исчезло ({e}). Переподключаюсь…"
                    )
                else:
                    self.get_logger().error(f"Ошибка чтения событий: {e}")

                with self.lock:
                    self.pressed_keys.clear()
                    # current_add и очередь команд НЕ чистим, чтобы уже сформированные команды не терять

                try:
                    self.device.close()
                except Exception:
                    pass
                self.device = None
                time.sleep(0.5)
                continue

            except Exception as e:
                self.get_logger().error(f"Неожиданная ошибка в read_events: {e}")
                with self.lock:
                    self.pressed_keys.clear()
                try:
                    self.device.close()
                except Exception:
                    pass
                self.device = None
                time.sleep(0.5)
                continue

        self.get_logger().info("Завершение потока чтения клавиатуры")

    # -------------------- обработка очереди команд --------------------

    def process_keys(self):
        """Обрабатывает очередь готовых команд и Esc."""
        if self.stop_event.is_set():
            return

        # проверяем Esc по текущему состоянию
        with self.lock:
            esc_pressed = ('Esc' in self.pressed_keys)

        if esc_pressed:
            self.get_logger().info("Escape pressed. Shutting down.")
            self.stop_event.set()
            self.destroy_node()
            rclpy.shutdown()
            return

        # извлекаем команду из очереди
        cmd = None
        with self.queue_lock:
            if self.command_queue:
                cmd = self.command_queue.popleft()

        if cmd is not None:
            msg = UInt8()
            msg.data = cmd
            self.publ_table_pos.publish(msg)
            print(cmd)

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
