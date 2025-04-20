"""
Управление любым функционалом с клавитаутры
"""

import evdev
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
import json
import copy


import sys, os
modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf


class DebugKeyboard(Node):
    def __init__(self):
        super().__init__('debug_keyboard')

        self.call_init_publ = self.create_publisher(Int8, 'run_vers_04_25', 42)
        self.start_finish_publ = self.create_publisher(Int8, 'start_finish', 3)
        self.border_publ = self.create_publisher(Int8, 'border_mode', 3)
        self.arm_publ = self.create_publisher(String, 'arm_action', 10)
        self.throw_short_publ = self.create_publisher(UInt8MultiArray, 'throw_short_mode', 10)
        self.img_publ = self.create_publisher(String, 'img_classif', 10)
        
        self.throw_short_subs = self.create_subscription(UInt8, 'short_throw_status', self.throw_short_callback, 10)
        #self.classific_subs = self.create_subscription(String, 'img_claasif', self.friut_callback, 5)

        self.arm_state = {value: key for key, value in conf.arm_states_table.items()}

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
            ecodes.KEY_A: 'a',
            ecodes.KEY_B: 'b',
            ecodes.KEY_T: 't',
            ecodes.KEY_S: 's',
            ecodes.KEY_I: 'i',
            ecodes.KEY_O: 'o',
            ecodes.KEY_C: 'c'
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
        self.dt = 1/200
        self.timer = self.create_timer(self.dt, self.process_keys)
        self.waiter = [0, 5/self.dt]



    def send_border_mode(self, number):
        assert isinstance(number, int)
        msg = Int8(data = number)
        self.border_publ.publish(msg)

    def send_arm_action(self, stroka):
        assert isinstance(stroka, str)
        msg = String(data = stroka)
        self.arm_publ.publish(msg)

    def throw_short_callback(self, mode_block):
        assert isinstance(mode_block.data, int)
        msg = UInt8MultiArray(data=[mode_block.data, 0])
        self.throw_short_publ.publish(msg)


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
        

        data = []
        pref = ''
        mode = -42
        
        priority_pref = ['i','o', 'b', 'a', 't', 's', 'c']  # Порядок приоритета
        pref = next((key for key in priority_pref if key in current_keys), None)

        priority_mode = ['0', '1', '2', '3', '4', '5', '6']
        mode = int(next((key for key in priority_mode if key in current_keys), -42))

        if pref:
            data = [pref, mode]

        if data:

            if data[0] == 'b':
                if self.waiter[0] == 0:
                    if data[1] in [0,1]:
                        msg = Int8(data = data[1])
                        self.border_publ.publish(msg)
                else:
                    self.waiter[1] += self.dt
                    if self.waiter[0] > self.waiter[1]:
                        self.waiter[0] = 0

            elif data[0] == 'a':
                if data[1] in [0,1,2,3,4,5,6]:
                    id_com = data[1]- 2
                    msg = String(data = self.arm_state[id_com])
                    self.get_logger().info(f'{self.arm_state[id_com]}')
                    self.arm_publ.publish(msg)
            

            elif data[0] == 't':
                msg = UInt8MultiArray()
                if data[1] in [1, 2, 3, 4, 5, 6]:
                    msg.data = [1, data[1]]
                elif data[1] == 0:
                    msg.data = [0, 0]
                if len(msg.data) > 0:
                    self.throw_short_publ.publish(msg)

            elif data[0] == 's':
                if data[1] in [0,1,2]:
                    msg = Int8(data = data[1]-1)
                    self.start_finish_publ.publish(msg)

            elif data[0] == 'i':
                msg = Int8(data = 1)
                self.call_init_publ.publish(msg)

            elif data[0] == 'o':
                msg = Int8(data = -1)
                self.call_init_publ.publish(msg)

            elif data[0] == 'c':
                img_data = {}
                if data[1] == 1:
                    img_data = {'class': 'sweet_pepper_good', 'x': 0.491}
                elif data[1] == 2:
                    img_data = {'class': 'pear_good', 'x': 0.521}
                elif data[1] == 3:
                    img_data = {'class': 'lemon_bad', 'x': 0.55}
                if img_data:
                    msg = String()
                    msg.data = json.dumps(img_data)
                    self.img_publ.publish(msg)
                    print(msg.data)




    def destroy_node(self):
        self.stop_event.set()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DebugKeyboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

