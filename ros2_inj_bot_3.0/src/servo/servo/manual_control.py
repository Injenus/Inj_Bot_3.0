import evdev
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt8

import glob
import errno
import time


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

        # Устройство пока не открываем жёстко — пусть read_events сам переподключается
        self.device = None

        # Состояние клавиш и блокировка для потокобезопасности
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

        # Паблишер команд
        self.publ_table_pos = self.create_publisher(UInt8, 'servo/lut', 3)

        # Поток для чтения событий клавиатуры
        self.read_thread = threading.Thread(target=self.read_events, daemon=True)
        self.read_thread.start()

        # Таймер для обработки нажатий
        self.timer = self.create_timer(1 / 200.0, self.process_keys)

    # -------------------- поиск клавиатуры --------------------

    def find_keyboard_device(self):
        """
        Поиск и выбор клавиатуры.
        Сначала пробуем стабильные пути /dev/input/by-id/*Keyboard*event-kbd,
        затем — перебор eventX с фильтром по нужным клавишам.
        """
        # 1. Пытаемся использовать стабильный by-id путь
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

        # 2. Fallback: перебор всех устройств ввода
        devices = [InputDevice(path) for path in evdev.list_devices()]

        self.get_logger().info("\nДоступные устройства ввода:")
        for i, dev in enumerate(devices):
            print(f"{i+1}. {dev.name} ({dev.path})")

        # Требуем хотя бы цифры 0–9 и Esc — фильтрует мыши, камеры и т.п.
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

    # -------------------- чтение событий с авто-переподключением --------------------

    def read_events(self):
        """Читает события клавиатуры в отдельном потоке с авто-reconnect."""
        self.get_logger().info("Старт потока чтения клавиатуры")
        backoff = 1.0

        while not self.stop_event.is_set():
            # Если устройства ещё нет или оно было сброшено — пытаемся найти
            if self.device is None:
                self.device = self.find_keyboard_device()
                if self.device is None:
                    self.get_logger().warn(
                        f"Клавиатура не найдена, повтор через {backoff:.1f} сек"
                    )
                    time.sleep(backoff)
                    backoff = min(backoff * 2.0, 10.0)
                    continue
                backoff = 1.0  # сброс backoff

            try:
                # Основной цикл чтения событий с текущего устройства
                for event in self.device.read_loop():
                    if self.stop_event.is_set():
                        break

                    if event.type == ecodes.EV_KEY:
                        with self.lock:
                            key_code = event.code
                            key_name = None

                            if key_code in self.arm_keys:
                                key_name = self.arm_keys[key_code]

                            if key_name is not None:
                                if event.value == 1:      # нажатие
                                    self.pressed_keys.add(key_name)
                                elif event.value == 0:    # отпускание
                                    self.pressed_keys.discard(key_name)

            except OSError as e:
                # Классический случай: устройство логически исчезло (ENODEV)
                if e.errno == errno.ENODEV:
                    self.get_logger().error(
                        f"Устройство ввода исчезло ({e}). Попробую переподключиться…"
                    )
                else:
                    self.get_logger().error(f"Ошибка чтения событий: {e}")

                # Сбрасываем состояние
                with self.lock:
                    self.pressed_keys.clear()

                try:
                    self.device.close()
                except Exception:
                    pass
                self.device = None

                time.sleep(1.0)
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
                time.sleep(1.0)
                continue

        self.get_logger().info("Завершение потока чтения клавиатуры")

    # -------------------- обработка клавиш --------------------

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

        # Сначала ищем букву (десятки)
        for key in current_keys:
            if key in self.letter_convert:
                add = self.letter_convert[key]
                break

        # Затем ищем цифру (единицы)
        for key in current_keys:
            try:
                key_int = int(key)
            except ValueError:
                continue

            data = key_int + add
            self.get_logger().info(f'Command: {data}')
            break

        if data is not None:
            lut_msg = UInt8()
            lut_msg.data = data
            self.publ_table_pos.publish(lut_msg)
            print(data)

    def destroy_node(self):
        self.stop_event.set()
        # Закрывать self.device напрямую не обязательно — read_events сам её закроет
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    detect_key = KeyboardNode()
    rclpy.spin(detect_key)
    detect_key.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
