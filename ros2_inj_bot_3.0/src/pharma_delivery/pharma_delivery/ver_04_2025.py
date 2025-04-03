"""
Робот находится на стартовой ячейке, боком к стенке, где qr-код, манипулятор уже смортит на эту стенку.
Робот распознает qr-код, парсит инфу - о движение (пока хуй занет - посдедовательность поворотов и нмоера палат??)

После чего наичнает движение. И ставит манипулятор строго вперёд на урвень для деткееции верхней аруки.

Сначала очвеидно едет прямо, смотрим по лидару на центральное показание и два ближайших боковых - стремиимся чтобы боковые были равны.
Когда подъедм на близкое растсояние следует повернуть, но будем двигаться БЕЗ повротов - омни колёса - учитвая, что лидар способен смоеть на все четыре стороны с центром и боковыми.
Таким образом двигаемся по всем повортоам.

Когда дотсигаем лекарств (хуй знает че делать), исходя из постоянства лабиринта, поворачиемся до тех пор, пока в нужна арука не будет в центре кадра.
Если арука в центре кадра, но передние два ближайших к центры разные -  занчит стоим криво. Поворачиемваемся до тех пор, пока расстония не будут примерно равны,
и дальше едем боком в нужную сторону пока нужна арука не будет в центре.
Едем на лекарстуво, пока лидар не покажет минимум (опрделённый). Захватлии лекарство.

Дальше вообще хуй знаетче делать - то ли по qr коду, то ли как хуй пойми. Но по идее логаика та же

используем конечный автомат
    (порядок движения записываем в список, двигаемся по нему)
    (контрольные расстония (для лабиринтаЮ, для апетки, для палаты) записаныв конфиграц. файле)
0 - стоим на старе, НЕ нашли qr, поворачиваем вправо руку из вертикальногог положения
1 - стоим на старте, нашли qr код, спраосили, повораисваем руку на прямо через промежуточное положение
2 - двигаемся согласно алгориму
3 - приехали в зону аптеки (короткий стоп)
4 - крутимся пока нужная арука не будет в центре камеры руки
5 - крутимся пока не станем перемендикуляор лекатсвам, запоминаем сторону вращения!!!
6 - смещаемся боком влевро если керутились по часовой стрелке (и наоброт) пока арука не будет в центре камеры
7 - едем до елкстра, доехали до лекарства
8 - короткая остановка
9 - отъехали назад для нормальных манёвров
10 - едем в палату - хз пока как и что

для каждого распознавания делам мини тряску по горзионтали!! иначе хуй распознаем

для w z - против часовой - ПЛЮС, по часовой - МИНУС
для lin y = вправо - МИНУС, влево - ПЛЮС
для lin x - вперёд ПЛЮС, назад - МИНУС

"""
### TODO: ДОБАВИТЬ МИКРО ПОДВИЖКИ ВО ВРЕМЯ РАСПОЗНАВАНИЙ !!!!!!!
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import json
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import copy
import numpy as np
import time

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'pharma_delivery', 'pharma_delivery')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config_04_2025 as conf

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from play_audio import play_audio


class PharmaDelivery(Node):
    def __init__(self):
        super().__init__("pharma_delivery")

        self.error = 0 # m  Л - П   елс иотрицатлеьная надо подворачивать налево, положит. - направо
        self.target_aruco = 0
        self.turn_direction_aruco = [0] # больше нулей - никакого не было  -1 - было против чаосво (влево),  1 - было поч асово (вправо)
        self.offset_direction_pharma = 0 # направление смещения до нужно аруки когда стоит перпендикулярно аптеке

        self.subs_aruco = self.create_subscription(String, 'aruco_markers', self.aruco_callback, 3)
        self.subs_qr_code = self.create_subscription(String, 'qr_codes', self.qr_code_callback, 3)
        self.subs_lidar = self.create_subscription(String, 'lidar/obstacles', self.lidar_callback, 3)

        self.publ_speed = self.create_publisher(Twist, '/cmd_vel', 3)
        self.publ_table_pos = self.create_publisher(UInt8, 'servo/lut', 3)
        
        self.period = 0.005
        self.timer = self.create_timer(self.period, self.loop)

        self.main_state = 0

        self.qr_data = {}
        self.aruco_data = {}
        self.lidar_all = {}
        self.lidar_basic = {}

        self.init_qr_info = {} # словарь хуй знает из чего
        self.turns = [] # пордок поворотов
        self.roadmap = ['front', 'right'] # фактичсекие перемещения (по идее равны поворотам только font в начале)

        self.arm_counter = [0, 0]
        self.turn_counter = 0
        self.stop_counter = 0
        self.shaking_timer = [0, 0] # 0 -left   1 - right второе значение - счетчик

        self.lidar_lock = False
        self.aruco_lock = False

        self.last_valid_basic_lidar = {}

        self.audio_flags = [True for i in range(42)] # соствлния карта флагов для послдеоватльности звуков, что чему соотвуеттсвует разобратьсчя придется не сразу, да, но мне пиздец как некогда
        # просто для КАЖДОЙ (даже если априоре она только 1 раз может) делаем флаг

    def qr_code_callback(self, msg):
        data = json.loads(msg.data)
        self.qr_data = {
            int(k): (
                tuple(v[0]),  # Преобразуем список в кортеж для центра
                [tuple(p) for p in v[1]],  # Список кортежей для точек
                v[2]  # Данные QR-кода
            ) for k, v in data.items()
        }
        self.get_logger().info(f'Получили: {self.qr_data}')

    def aruco_callback(self, msg):
        if not self.aruco_lock:
            data = json.loads(msg.data)
            self.aruco_data = {
                int(k): (
                    tuple(v[0]),
                    tuple(tuple(p) for p in v[1]),
                    int(v[2])
                ) for k, v in data.items()
            }
            #self.aruco_data = copy.deepcopy(data)
            self.get_logger().info(f'Получили: {self.aruco_data}')

    def lidar_callback(self, msg):
        if not self.lidar_lock:
            data = json.loads(msg.data)
            data = {int(k): v for k, v in data.items()}
            #print(data)
            angles = np.array(sorted(data.keys(), key=lambda x: float(x)), dtype=np.float32)
            # distances = np.array([self.data[str(int(angle))] for angle in angles], dtype=np.float32)
            angle_step = round(np.abs(angles[1] - angles[0]))

            self.lidar_all = copy.deepcopy(data)
            self.lidar_basic = {
                'front': [data[360-angle_step], data[0], data[0+angle_step]],
                'right': [data[90-angle_step], data[90], data[90+angle_step]],
                'back': [data[180-angle_step], data[180], data[180+angle_step]],
                'left': [data[270-angle_step], data[270], data[270+angle_step]],
                'front_aruco': [data[360-2*angle_step], data[0], data[0+2*angle_step]]
            }
            self.get_logger().info(f'Basic: {self.lidar_basic}')

    def get_turns(self):
        #self.turns = self.init_qr_info
        pass

    def get_roadmap(self):
        #self.roadmap = self.turns
        pass

    def servo_shaking(self, mode):
        mapping = {False: int(2*str(mode)), True: int(3*str(mode))}

        if self.shaking_timer[1] == False:
            if self.shaking_timer[0] == 0:
                msg = UInt8()
                msg.data = mapping[self.shaking_timer[1]]
                self.publ_table_pos.publish(msg)
                self.shaking_timer[0] += 1
            else:
                self.shaking_timer[0] += 1
                if self.shaking_timer[0] >= 1/(self.period):
                    self.shaking_timer[1] = True
                    self.shaking_timer[0] = 0
        else:
            if self.shaking_timer[0] == 0:
                msg = UInt8()
                msg.data = msg.data = mapping[self.shaking_timer[1]]
                self.publ_table_pos.publish(msg)
                self.shaking_timer[0] += 1
            else:
                self.shaking_timer[0] += 1
                if self.shaking_timer[0] >= 1/(self.period):
                    self.shaking_timer[1] = False
                    self.shaking_timer[0] = 0

    def play_with_flags(self, name, idx):
        if self.audio_flags[idx]:
            play_audio(name)
            self.audio_flags[idx] = False


    def loop(self):
        if self.main_state == 0:
            msg = UInt8()
            msg.data = 1
            self.publ_table_pos.publish(msg)
        if self.qr_data:
            if len(self.qr_data) == 1:
                if self.qr_data[0][2]:
                    self.play_with_flags('qr_code.wav', 0)
                    self.init_qr_info = copy.deepcopy(self.qr_data)
                    self.get_turns()
                    self.get_roadmap()
                    self.main_state = 1
                else:
                    self.servo_shaking(1)                
                    print('data is empty')
            else:
                print(f'too much {len(self.qr_data)}')


        if self.main_state == 1 or self.main_state == 2:
            self.main_state = 2
            if self.arm_counter[0] < 2/self.period:
                self.arm_counter[0] += 1
                msg = UInt8()
                msg.data = 2
                self.publ_table_pos.publish(msg)
            elif self.arm_counter[1] < 2/self.period:
                self.arm_counter[1] += 1
                msg = UInt8()
                msg.data = 3
                self.publ_table_pos.publish(msg)
        
        
        if self.main_state == 2:
            self.lidar_lock = True
            # TODO !!!!!! реашить инкрементацию последователньости повротов и напврвлений !!!
            key = self.roadmap[self.turn_counter]
            if key in ['front', 'back']:
                self.error = self.lidar_basic[key][0] - self.lidar_basic[key][2]
            if key == 'left':
                self.error = self.lidar_basic['left'][1] * 1.00375 - self.lidar_basic['left'][2] # 1.00375 СТРОГО ДЛЯ 10град.!!!!!!!!
            if key == 'right':
                self.error = self.lidar_basic['right'][0] - self.lidar_basic['right'][1] * 1.00375 # компенсация что мы сравниваем не крайние а средний с краним, у котрого длнеа априори меньше

            if self.turn_counter == 0: # едем передом
                self.play_with_flags('polnyi_vpered.wav', 1)
                linear_x = conf.LIN_X_SPEED
                linear_y = 0
                angular_z = -self.error * conf.P_koef # по часвово должно быть -1, А така как ошибка плюс - доб. занк минус
                
            elif self.turn_counter == 1: #едем правым бортом
                self.play_with_flags('pravo_rulia.wav', 2)
                linear_x = 0
                linear_y = -conf.LIN_Y_SPEED
                angular_z = -self.error * conf.P_koef # против чаосв - должно быть плюс , ошибка меньше нуля - доб. минус
            


            self.lidar_lock = False

            if self.lidar_basic[key][1] <= conf.lidar_offsets[key] + conf.wall_distance:
                self.turn_counter += 1
                linear_x = 0
                linear_y = 0
                angular_z = 0

            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.linear.y = linear_y
            twist_msg.angular.z = angular_z
            self.twist_pub.publish(twist_msg)
            self.get_logger().debug(f"PUB: lin_x={linear_x}, lin_y={linear_y}, ang_z={angular_z}")

            if self.turn_counter == len(self.roadmap):
                self.main_state == 3
                

        if self.main_state == 3:
            self.stop_counter += 1
            if self.stop_counter >= conf.confirm_time/self.period:
                self.main_state = 4
                self.stop_counter = 0


        if self.main_state == 4:
            angular_z = -conf.ANG_Z_SPEED / 2  # просто предолоэение что изначальнор нужно екрится по часовой
            self.aruco_lock = True
            if self.aruco_data:
                for key, val in self.aruco_data.items():
                    if key == self.target_aruco:
                        err = 0.5 - val[0][0]
                        if abs(err) <= conf.thresh_taking_pharma_err:  # арука прмиерно в центре
                            self.main_state = 5
                            angular_z = 0
                        elif err > 0:
                            angular_z = conf.ANG_Z_SPEED
                        elif err < 0:
                            angular_z = -conf.ANG_Z_SPEED

            self.aruco_lock = False

            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            self.twist_pub.publish(twist_msg)
            self.get_logger().debug(f"Aruco ang_z={angular_z}")


        if self.main_state == 5:
            self.aruco_lock = True
            self.lidar_lock = True

            key = 'front_aruco'
            error = self.lidar_basic[key][0] - self.lidar_basic[key][2]
            if abs(error) <= conf.thresh_lidar_pharma:
                angular_z = 0
                self.main_state = 6
            elif error > 0: # левый луч длинне правого -  надо вращаться по чаововой - вправо
                angular_z = -conf.ANG_Z_SPEED / 3
                self.turn_direction_aruco.append(1)
            elif error < 0:
                angular_z = conf.ANG_Z_SPEED / 3
                self.turn_direction_aruco.append(-1)
            
            self.aruco_lock = False
            self.lidar_lock = False

            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            self.twist_pub.publish(twist_msg)
            self.get_logger().debug(f"Pharma lidar ang_z={angular_z}")

            self.offset_direction_pharma = max(set(self.turn_direction_aruco), key=self.turn_direction_aruco.count) # нашли каких элиентов больше всего
            

        if self.main_state == 6:
            if self.offset_direction_pharma == 0: # стоим уже ровно
                self.main_state = 7
            elif self.offset_direction_pharma == -1: # крутилися против часовой тогда смещаемся вправо
                linear_y = -conf.LIN_Y_SPEED / 2
            elif self.offset_direction_pharma == 1:
                linear_y  = conf.LIN_Y_SPEED / 2

            self.aruco_lock = True
            for key, val in self.aruco_data.items():
                if key == self.target_aruco:
                    err = 0.5 - val[0][0]
                    if abs(err) <= conf.thresh_taking_pharma_err:  # арука прмиерно в центре
                        self.main_state = 7
                    elif err > 0: # надо ехать влево
                        linear_y  = conf.LIN_Y_SPEED / 2
                    elif err < 0:
                        linear_y  = -conf.LIN_Y_SPEED / 2
            self.aruco_lock = False
            
            twist_msg = Twist()
            twist_msg.linear.y = linear_y
            self.twist_pub.publish(twist_msg)
            self.get_logger().debug(f"Pharma aruco Y={linear_y}")


        if self.main_state == 7:
            self.lidar_lock = True
            if self.lidar_basic['front'][1] > conf.pharma_distance:
                linear_x  = conf.LIN_X_SPEED
            else:
                linear_x = 0
                self.main_state = 8

            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            self.twist_pub.publish(twist_msg)
            self.get_logger().info(f'Go to cure X={linear_x}')
            self.lidar_lock = False


        if self.main_state == 8:
            self.stop_counter += 1
            if self.stop_counter >= conf.confirm_time/self.period:
                self.main_state = 9


        if self.main_state == 9:
            self.lidar_lock = True
            if self.lidar_basic[0] < conf.pharma_for_back_dictance:
                linear_x = -conf.LIN_X_SPEED
            else:
                linear_x = 0
                self.main_state = 10
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            self.publ_speed.publish(twist_msg)

        if self.main_state == 10:
            print('А ЧЕ ДЕЛАТЬ????')
            
            



            



            




    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    pharma_delivery = PharmaDelivery()
    rclpy.spin(pharma_delivery)
    pharma_delivery.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

                                    








                           




