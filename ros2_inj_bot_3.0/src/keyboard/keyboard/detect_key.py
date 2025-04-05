import evdev
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
import threading
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
import copy
import sys
import os
import numpy as np
from matplotlib.path import Path
import time

receive_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'arm_n_cam_mount_api_8_bytes')
if receive_data_path not in sys.path:
    sys.path.append(receive_data_path)

from arm_full_pack import *

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('detect_key')

        self.LINEAR_SPEED_X = 0.1 # м/с
        self.LINEAR_SPEED_Y = 0.05  # м/с
        self.ANGULAR_SPEED = 3.0 # рад/с

        with open(os.path.join(receive_data_path, 'config_arm.json'), 'r') as file:
            self.config_arm = json.load(file)
            # l1, l2, l3 = config['length']
            # ang_range_arm = config['ang_range']

        with open(os.path.join(receive_data_path, 'config_cam.json'), 'r') as file:
            self.config_cam = json.load(file)
            # l1, l2, l3 = config['length']
            # ang_range_arm = config['ang_range']

        with open(os.path.join(receive_data_path, 'config_grip.json'), 'r') as file:
            self.config_grip = json.load(file)

        actual_ang_range = np.deg2rad([[a - off, b - off] for (a, b), off in zip(self.config_arm['ang_range'], self.config_arm['offset'])])
        valid_area_data = np.load(os.path.join(receive_data_path, 'valid_area_data.npz'))
        specific_theta_angles = valid_area_data['main']
        specific_valid_area = [Path(valid_area_data[f'array_{i}']) for i in range(len(specific_theta_angles.tolist()))]
        global_valid_area = Path(np.array([[-np.inf, -np.inf], [-np.inf, -170], [-210, -170], [-210, 0], [0, 0], [210, 0], [210, -170], [np.inf, -170], [np.inf, -np.inf]]))

        self.arm = ArmIKP(self.config_arm['length'], actual_ang_range, specific_valid_area, specific_theta_angles, global_valid_area)

        self.SERVO_FREQ = 100
        self.SERVO_MIN_LINEAR_STEP = 10 # мм, всё что меньше НЕ считается изменением текущих линейных (xyz, rh) координат
        self.SERVO_MIN_ANG_STEP = 3 # град., всё что меньше НЕ считается изменениеми текущих угловых координат (theta)

        self.SERVO_DEG_PER_SEC = 15
        self.SERVO_ANG_STEP = self.SERVO_DEG_PER_SEC / self.SERVO_FREQ

        self.SERVO_MM_PER_SEC = 40
        self.SERVO_LINEAR_STEP = self.SERVO_MM_PER_SEC / self.SERVO_FREQ

        self.ARM_RANGE = self.config_arm['ang_range']
        self.ARM_OFFSET = self.config_arm['offset']

        self.GRIP_RANGE = self.config_grip['ang_range']
        self.GRIP_STATE = self.config_grip['ang_range']

        self.CAM_RANGE = self.config_cam['ang_range']
        self.CAM_OFFSET = self.config_cam['offset']
    
        self.last_arm_pos = copy.deepcopy([0,0,0,0]) # это в реальных (-..+) углах каждого сервопривода! - обобщённые координаты, напрямую использующиеся в ОЗК и ПЗК
        self.current_gener_coord = copy.deepcopy(self.last_arm_pos) # инициализация текущих обобщ. координат - изменются вследствие тиков узла

        self.is_polar = True

        self.last_xyz = self.arm.dkp_2dof(np.deg2rad(self.last_arm_pos))
        self.last_cyclin = ArmIKP.cartesian_to_cylindrical(self.last_xyz, np.deg2rad([0,0,0,0])) # theta r h
        print('lc', self.last_cyclin)
        self.last_cyclin[0] = np.rad2deg(self.last_cyclin[0]) 

        self.last_grip = 0 #None
        self.last_cam = [0, 0] #[None, None] # нулями потому что 0 невозможен  а значит это заведеоме непорпделенное значение и костыль ели данных ещё не было

        self.current_xyz = copy.deepcopy(self.last_xyz)
        self.currnet_cyclin = copy.deepcopy(self.last_cyclin) # theta, r, h

        self.current_grip = copy.deepcopy(self.last_grip)
        self.current_cam = copy.deepcopy(self.last_cam)
        
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

        self.value_lock = False #  блокируте изменение клавиш из пришедених данных, пока идёт цикл
        self.update_timer, self.update_period = time.time(), 5.

    def get_last_servo_pos(self, msg):
        all_data = list(msg.data)
        # получаем актуальные координаты от драйвера - самые актуальные реальные данные, а значит, неважно, что мы там нассчитали с текущими, они тоже становятся здесь равны актуальным
        self.last_arm_pos = [x - y for x, y in zip(all_data[:4], self.ARM_OFFSET)] # приводим к отрицательному виду (полному виду, реальный дипазон грудсов)
        #print(self.last_arm_pos)
        self.last_xyz = self.arm.dkp_2dof(np.deg2rad(self.last_arm_pos))
        #print(self.last_xyz)
        self.last_cyclin = ArmIKP.cartesian_to_cylindrical(self.last_xyz, np.deg2rad(self.ARM_RANGE[0])) # theta r h
        self.last_cyclin[0] = np.rad2deg(self.last_cyclin[0])
        self.last_grip = all_data[4]
        self.last_cam = [x-y for x, y in zip(all_data[5:], self.CAM_OFFSET)]
        # # необходимо использовать текущие координаты вместе с последнирми, потому что в общем случае скорость измения текущих вследжии тиков узла может бьть достаточно большой, чтобы текущие координаты зщначительно отличались от послшдених
        if time.time() - self.update_timer > self.update_period:
            self.current_gener_coord = copy.deepcopy(self.last_arm_pos)
            self.current_xyz = copy.deepcopy(self.last_xyz)
            self.currnet_cyclin = copy.deepcopy(self.last_cyclin)
            self.current_grip = copy.deepcopy(self.last_grip)
            self.current_cam = copy.deepcopy(self.last_cam)
            self.update_timer = time.time()


    def check_difference(self):
        self.cyclin_is_diff =  abs(self.currnet_cyclin[0] - self.last_cyclin[0]) > self.SERVO_MIN_ANG_STEP or\
        abs(self.currnet_cyclin[1] - self.last_cyclin[1]) > self.SERVO_MIN_LINEAR_STEP or\
        abs(self.currnet_cyclin[2] - self.last_cyclin[2]) > self.SERVO_MIN_LINEAR_STEP
        
        self.cartesian_is_diff = abs(self.current_xyz[0] - self.last_xyz[0]) > self.SERVO_MIN_LINEAR_STEP or\
        abs(self.current_xyz[1] - self.last_xyz[1]) > self.SERVO_MIN_LINEAR_STEP or\
        abs(self.current_xyz[2] - self.last_xyz[2]) > self.SERVO_MIN_LINEAR_STEP

        self.grip_is_diff = self.current_grip != self.last_grip

        self.cam_is_diff = abs(self.current_cam[0] - self.last_cam[0]) > self.SERVO_MIN_ANG_STEP or\
        abs(self.current_cam[1] - self.last_cam[1]) > self.SERVO_MIN_ANG_STEP



    # def find_keyboard_device(self):
    #     """Находит устройство клавиатуры, поддерживающее нужные клавиши."""
    #     # Объединяем все клавиши из трех групп
    #     self.all_keys = {
    #         **self.wheel_keys,
    #         **self.cam_keys,
    #         **self.arm_keys
    #     }
    #     devices = [InputDevice(path) for path in evdev.list_devices()]
    #     for device in devices:
    #         try:
    #             caps = device.capabilities()
    #             if ecodes.EV_KEY in caps:
    #                 available_keys = caps[ecodes.EV_KEY]
    #                 # Проверяем наличие хотя бы одной нужной клавиши
    #                 if any(key in self.all_keys for key in available_keys):
    #                     return device
    #         except (OSError, PermissionError):
    #             continue
    #     return None
    

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
                    required_keys = {ecodes.KEY_A, ecodes.KEY_P, ecodes.KEY_S, ecodes.KEY_N, ecodes.KEY_Q, ecodes.KEY_B, ecodes.KEY_ESC}
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
            print('w')
        elif 's' in current_keys and 'w' not in current_keys:
            linear_x = -self.LINEAR_SPEED_X
            print('s')
        
        # Движение по Y (A/D)
        if 'a' in current_keys and 'd' not in current_keys:
            linear_y = self.LINEAR_SPEED_Y
            print('a')
        elif 'd' in current_keys and 'a' not in current_keys:
            linear_y = -self.LINEAR_SPEED_Y
            print('d')
        
        # Вращение по Z (Q/E)
        if 'q' in current_keys and 'e' not in current_keys:
            angular_z = self.ANGULAR_SPEED
            print('q')
        elif 'e' in current_keys and 'q' not in current_keys:
            angular_z = -self.ANGULAR_SPEED
            print('e')

        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        
        self.twist_pub.publish(twist_msg)
        self.get_logger().debug(
            f"PUB: lin_x={linear_x}, lin_y={linear_y}, ang_z={angular_z}"
        )
        if 0:
            #-----------------------------------------------------
            """
            4-6 theta или x
            2-8 h или z
            1-7 r или y
            3-9 захват
            0-5 цилиндрические или декартовые 
            """
            # переключение типа координат
            if '0' in current_keys and '5' not in current_keys:
                print(0)
                self.is_polar = True
            elif '5' in current_keys and '0' not in current_keys:
                self.is_polar = False
                print(5)

            if self.is_polar:
                # вращение базы манипулятора
                if '4' in current_keys and '6' not in current_keys:
                    self.currnet_cyclin[0] -= self.SERVO_ANG_STEP
                elif '6' in current_keys and '4' not in current_keys:
                    self.currnet_cyclin[0] += self.SERVO_ANG_STEP
                # изменение высоты
                if '2' in current_keys and '8' not in current_keys:
                    self.currnet_cyclin[2] -= self.SERVO_LINEAR_STEP
                elif '8' in current_keys and '2' not in current_keys:
                    self.currn#-----------------------------------------------------
            """
            4-6 theta или x
            2-8 h или z
            1-7 r или y
            3-9 захват
            0-5 цилиндрические или декартовые 
            """
            # переключение типа координат
            if '0' in current_keys and '5' not in current_keys:
                print(0)
                self.is_polar = True
            elif '5' in current_keys and '0' not in current_keys:
                self.is_polar = False

                # изменение расстояния
                if '1' in current_keys and '7' not in current_keys:
                    self.currnet_cyclin[1] -= self.SERVO_LINEAR_STEP
                elif '7' in current_keys and '1' not in current_keys:
                    self.currnet_cyclin[1] += self.SERVO_LINEAR_STEP
            else:
                # x
                if '4' in current_keys and '6' not in current_keys:
                    self.current_xyz[0] -= self.SERVO_LINEAR_STEP
                elif '6' in current_keys and '4' not in current_keys:
                    self.currnet_cyclin[0] += self.SERVO_ANG_STEP
                # z
                if '2' in current_keys and '8' not in current_keys:
                    self.currnet_cyclin[2] -= self.SERVO_LINEAR_STEP
                elif '8' in current_keys and '2' not in current_keys:
                    self.currnet_cyclin[2] += self.SERVO_LINEAR_STEP
                # y
                if '1' in current_keys and '7' not in current_keys:
                    self.currnet_cyclin[1] -= self.SERVO_LINEAR_STEP
                elif '7' in current_keys and '1' not in current_keys:
                    self.currnet_cyclin[1] += self.SERVO_LINEAR_STEP


            if '3' in current_keys and '9' not in current_keys:
                self.current_grip = 1
            elif '9' in current_keys and '3' not in current_keys:
                self.current_grip = 0


            if 'left' in current_keys and 'right' not in current_keys:
                self.current_cam[0] -= self.SERVO_MIN_ANG_STEP
            elif 'right' in current_keys and 'left' not in current_keys:
                self.current_cam[0] += self.SERVO_MIN_ANG_STEP

            if 'up' in current_keys and 'down' not in current_keys:
                self.current_cam[1] -= self.SERVO_MIN_ANG_STEP
            elif 'down' in current_keys and 'up' not in current_keys:
                self.current_cam[1] += self.SERVO_MIN_ANG_STEP

            self.check_difference()
            self.msg_data = []

            arm_success = False
            if self.cyclin_is_diff:
                result = np.rad2deg(self.arm.ikp_2dof(ArmIKP.cylindrical_to_cartesian(np.deg2rad(self.currnet_cyclin))))
                arm_success = result is not None
            elif self.cartesian_is_diff:
                result = np.rad2deg(self.arm.ikp_2dof(self.current_xyz))
                arm_success = result is not None

            if arm_success:
                self.current_gener_coord = result
                self.msg_data = copy.deepcopy([round(el) + self.ARM_OFFSET[i] for i, el in enumerate(self.current_gener_coord)])
            else:
                self.msg_data = copy.deepcopy(self.last_arm_pos)

            self.msg_data += [self.current_grip]

            if self.cam_is_diff:
                self.msg_data += [round(el) + self.CAM_OFFSET[i] for i, el in enumerate(self.current_cam)]
            else:
                self.msg_data += self.last_cam

            # print(self.current_gener_coord)
            # print(self.current_grip)
            # print(self.current_cam)

            if arm_success or self.grip_is_diff or self.cam_is_diff:
                print('af r', self.msg_data)
                clipper = self.ARM_RANGE + self.GRIP_RANGE + self.CAM_RANGE
                self.msg_data = np.clip(self.msg_data, [c[0] for c in clipper], [c[1] for c in clipper]).tolist()

                servo_msg = UInt8MultiArray()
                servo_msg.data = bytes(self.current_gener_coord + [self.current_grip] + self.current_cam)
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