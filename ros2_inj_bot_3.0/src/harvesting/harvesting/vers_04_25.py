"""
пакет greenhouse_simple
узлы:
    - узел публикация результат поиска овощзей (список из списков [класс, [относительные координаты]])
    - нехависимый узел движения вдоль бордюра с логикой поворота (приманет команды стоп/продолжить/убиться)
    - узел выполднения скриптов манипулятором (сбить/собрать/бросить1/бросить2)
    - узел движения к коробке - маршрут движения/убиться(?)
    - узел движения к финишу от коробки  - маршрут движения/убиться(?)
    - узел координатор:


на обычных колёсах едем вдоль бордюра
    едем по бюоковому лучу лидара - т.к. колёса обычные - вбок не сносит - единсвтенное отключение - из-за поворота корпуса
        так что компенсируем откллонения расстояния от целевого вращением

    когда нужно повернуть (определяем по растсонияю перднего луча),
    начинаем поврот, и крутим дло тех пор, пока бюокове растсоние снова не станет целевым
так едем до тех пор, пока не увидим шестой овощ

манипулятор сразу ставим в бок на высоту (вот хз - либо высота сердеины овоща, либо магнита, либо будет полдстройка)
(учёт, что будет сачок)
параллельно с ездой детектируем овощи:
    1) увидели врагмент овоща - возможно распознался как несколько - игнор
    2) увидели овощ целоком, один - едем пок не булет в центре
    3)индекс овощей увелчи
    4) в центре, сбиваем, пока не пропадёт (должен остутвовать на n (5) подряд кадрах)
    5) едем дальше

    если увидели целевой овощ - скрипт получения в сачок (
        вниз, пока не пропадёт из кадров (5)
        вперёд на эмпиричсекую величину
        вверх пока не будет в центре
        вверх пока не пропадёт из кадров (лиюо веверх пока не будет внизу на кукю-то долю)
        назад на эмпиричсекую величину
    )

    если овощ сбивать не надо, то просто едем дальше, увеличивая индекс на 1:
        1) увидели врагмент овоща - возможно распознался как несколько - игнор
        2) увидели овощ целоком, один - едем пок не булет в центре
        3)индекс овощей увелчи
        4) в центре
        5) едем дальше

(когда приедем к шестому овощу убиваем узел бордюра - едем к коробке
приехали, бросили
поехали на финиш)

-------------------------------------------
есть измения в алгоримте смотрте код
кртако
    всегда едем вдоль  ордюра
    видим офощ, стоп!
    сенс овоща - либо пох, либо сбить, либо взять
        еслит НЕ взять, продолжаем ехать
        еслди взять - берём, езда стоп, маршрут к боксу актив
        доехали кинул, вернулись, муршрут стоп, езда актив

    а так же
    имеем счётчик сенса овощей от 1(0) до 6
    если иеем 6, то сразу смотрим на заднее растояние, и стоп езда по нему. всё.
    
"""

"""
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
import json
import copy



import sys, os
modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        self.border_publ = self.create.publisher(Int8, 'border_mode', 3)
        self.arm_publ = self.create.publisher(String, 'arm_action', 10)
        self.throw_short_publ = self.create.publisher(UInt8MultiArray, 'throw_short_mode', 10)
        
        self.throw_short_subs = self.create_subscription(UInt8, 'short_throw_status', self.throw_short_callback, 10)
        self.classific_subs = self.create_subscription(String, 'img_claasif', self.friut_callback, 5)

        self.timer = self.create_timer(conf.dt, self.loop)

        self.border_mode = 0
        self.arm_command = ''
        self.throw_short_mode = [0, 0]

        self.fruit_classif = {}
        self.fruit_lock = Lock()
        self.throw_short_status = 0

        
    def friut_callback(self, msg):
        with self.fruit_lock:
            self.fruit_classif = json.loads(msg.data)

    def throw_short_callback(self, msg):
        self.throw_short_status = msg.data


    def send_border_mode(self, number):
        assert isinstance(number, int)
        msg = Int8(data = number)
        self.border_publ.publish(msg)

    def send_arm_action(self, stroka):
        assert isinstance(stroka, str)
        msg = String(data = stroka)
        self.arm_publ.publish(msg)

    def send_throw_short_mode(self, mode_block):
        assert isinstance(mode_block, list)
        assert len(mode_block) == 2
        msg = UInt8MultiArray(data=mode_block)
        self.throw_short_publ.publish(msg)


    def loop(self):
        pass