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


class PharmaDelivery(Node):
    def __init__(self):
        super().__init__("pharma_delivery")

        self.LIN_X_SPEED = 0.3
        self.LIN_Y_SPEED = 0.3
        self.ANG_Z_SPEED = 1.5
        self.P_koef = 0.1
        self.error = 0 # cm

        self.subs_aruco = self.create_subscription(String, 'aruco_markers', self.aruco_callback, 1)
        self.subs_qr_code = self.create_subscription(String, 'qr_codes', self.qr_code_callback, 1)
        self.subs_lidar = self.create_subscription()

        self.publ_speed = self.create_publisher(Twist, '/cmd_vel', 1)
        self.publ_table_pos = self.create_publisher(UInt8, 'servo/lut', 1)
        
        self.timer = self.create_timer(0.005, self.calculate_speed)