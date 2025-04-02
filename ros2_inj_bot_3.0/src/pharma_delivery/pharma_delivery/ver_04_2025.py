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
4 - крутимся пока не станем перпендикулярно декарствам
5 - движемся к лекарству, центрируясь по его аруке
6 - доехали до лекарства, короткая остановка
7 - отъехали назад для нормальных манёвров
8 - едем в палату - хз пока как и что

"""

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import json
from std_msgs.msg import UInt8
import copy

import config_04_2025 as conf


class PharmaDelivery(Node):
    def __init__(self):
        super().__init__("pharma_delivery")

        self.error = 0 # m  Л - П   елс иотрицатлеьная надо подворачивать налево, положит. - направо
        self.target_aruco = 0

        self.subs_aruco = self.create_subscription(String, 'aruco_markers', self.aruco_callback, 3)
        self.subs_qr_code = self.create_subscription(String, 'qr_codes', self.qr_code_callback, 3)
        self.subs_lidar = self.create_subscription(String, 'lidar/obstacles', self.lidar_callback 3)

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
        self.roadmap = ['font', 'right'] # фактичсекие перемещения (по идее равны поворотам только font в начале)

        self.arm_counter = [0, 0]
        self.turn_counter = 0
        self.stop_counter = 0

        self.lidar_lock = False
        self.aruco_lock = False


        def qr_code_callback(self, msg):
            data = json.loads(msg.data)
            self.qr_data = {
                int(k): (
                    tuple(v[0]), (tuple(p) for p in v[1]), v[2]
                ) for k, v in data.items()
            }
            #self.qr_data = copy.deepcopy(data)
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
                # angles = np.array(sorted(self.data.keys(), key=lambda x: float(x)), dtype=np.float32)
                # distances = np.array([self.data[str(int(angle))] for angle in angles], dtype=np.float32)
                angle_step = np.abs(angles[1] - angles[0])

                self.lidar_all = data
                self.lidar_basic = {
                    'front': [data[360.-angle_step], data[0.], data[0.+angle_step]],
                    'right': [data[90.-angle_step], data[90.], data[90.+angle_step]],
                    'back': [data[180.-angle_step], data[180.], data[180.+angle_step]],
                    'left': [data[270.-angle_step], data[270.], data[270.+angle_step]]
                }

        def get_turns(self):
            #self.turns = self.init_qr_info
            pass

        def get_roadmap(self):
            #self.roadmap = self.turns
            pass


        def loop(self):
            if self.main_state == 0:
                msg = UInt8()
                msg.data = 1
                self.publ_table_pos.publish(msg)
            if self.qr_data:
                if len(self.qr_data) == 1:
                    if self.qr_data[2]:
                        self.init_qr_info = copy.deepcopy(self.qr_data)
                        self.get_turns()
                        self.get_roadmap()
                        self.main_state = 1
                    else:
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
                
                key = self.roadmap[self.turn_counter]
                self.error = self.lidar_basic[key][0] - self.lidar_basic[key][2]

                if self.turn_counter == 0: # едем передом
                    linear_x = conf.LIN_X_SPEED
                    linear_y = 0
                    angular_z = -self.error * conf.P_koef
                    
                elif self.turn_counter == 1: #едем правым бортом
                    linear_x = 0
                    linear_y = -conf.LIN_Y_SPEED
                    angular_z = -self.error * conf.P_koef
                


                self.lidar_lock = False

                if self.lidar_basic[key][1] <= conf.lidar_offsets[key] + conf.wall_distances:
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
                time.sleep(self.period/2)
                self.stop_counter += 1
                if self.stop_counter >= conf.confirm_time/(self.period/2):
                    self.main_state = 4

            if self.main_state == 4:
                pass

            if self.main_state == 5:
                self.aruco_lock = True
                if self.aruco_data:
                    for key, value in self.aruco_data.items():
                        if key == self.target_aruco:
                            real_relative_x = value [0][0]
                            err = 0.5 - X
                            if abs(err) > conf.thresh_taking_pharma_err:
                                linear_y = conf.LIN_Y_SPEED * err/abs(err) / 1.5
                            else:
                                linear_y = 0

                            linear_x = 0
                            angular_z = 0
                            twist_msg = Twist()
                            twist_msg.linear.x = linear_x
                            twist_msg.linear.y = linear_y
                            twist_msg.angular.z = angular_z
                            self.twist_pub.publish(twist_msg)
                            self.get_logger().debug(f"pun_pharma_get: lin_x={linear_x}, lin_y={linear_y}, ang_z={angular_z}")

                else:
                    print('нет арук')


                self.aruco_lock = False

                                    








                           




