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
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
from threading import Thread, Event, Lock
import json
import copy
import time
from datetime import datetime

import sys, os
modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'ros2_inj_bot_3.0', 'src', 'harvesting', 'harvesting')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

import config as conf

modules_data_path = os.path.join(os.path.expanduser('~'), 'Inj_Bot_3.0', 'modules')
if modules_data_path not in sys.path:
    sys.path.append(modules_data_path)

from play_audio import play_audio


def get_time():
    return datetime.now().strftime("%H:%M:%S")

def write_log(log_string):
    print(log_string)
    with open('/home/inj/Inj_Bot_3.0/ros2_inj_bot_3.0/src/harvesting/harvesting/log.txt', 'a') as f:
        f.write(log_string)


class ManagedThread:
    def __init__(self, name, node, target_function=None, args=(), parent=None):
        self.name = name
        self.node = node
        self.parent = parent
        self.target_function = target_function
        self.args = args
        self.thread = None
        self.running = False
        #self.paused = Event()
        self.child = None
        self.lock = Lock()
        #self.paused.clear()
        self.done_event = Event()
        self.done_event.clear()

    def start(self):
        with self.lock:
            if not self.running and not self.thread:
                self.running = True
                self.thread = Thread(target=self.run, daemon=True)
                self.thread.start()
                self.node.get_logger().info(f"{self.name} started")

    def run(self):
        try:
            if self.target_function:
                self.target_function(self, *self.args)
        except Exception as e:
            self.node.get_logger().error(f"Error in {self.name}: {str(e)}")
        finally:
            self.done_event.set()
            self.stop()

    def stop(self):
        with self.lock:
            if self.running:
                self.running = False
                if self.child:
                    self.child.stop()
                # if self.parent and self.parent.paused.is_set():
                #     self.parent.resume()
                self.thread = None
                self.node.get_logger().info(f"{self.name} stopped")
        

    def resume(self):
        with self.lock:
            self.paused.clear()
            self.node.get_logger().info(f"{self.name} resumed")

    def start_child(self, target_function, args=()):
        with self.lock:
            if not self.child and self.running:
                #self.paused.set()
                self.child = ManagedThread(
                    name=f"{self.name}_child",
                    node=self.node,
                    target_function=target_function,
                    args=args,
                    parent=self
                )
                self.child.start()
                self.child.done_event.wait()
                self.child = None
                #self.node.get_logger().info(f"{self.name} paused, child {self.child.name} started")


class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        self.border_publ = self.create_publisher(Int8, 'border_mode', 3)
        self.arm_publ = self.create_publisher(String, 'arm_action', 10)
        self.throw_short_publ = self.create_publisher(UInt8MultiArray, 'throw_short_mode', 10)
        self.start_finish_publ = self.create_publisher(Int8, 'start_finish', 10)
        self.emergency_stop_publ = self.create_publisher(Twist, '/cmd_vel', 100)
        
        self.throw_short_subs = self.create_subscription(UInt8, 'short_throw_status', self.throw_short_callback, 10)
        self.classific_subs = self.create_subscription(String, 'img_classif', self.friut_callback, 10)
        self.call_init = self.create_subscription(Int8, 'run_vers_04_25', self.init_callback, 42)

        # Разделяемые ресурсы
        self.fruit_classif = {}
        self.throw_short_status = 0
        self.init_command = 0
        self.count_blocks = 0

        self.last_fruit = ''
        self.was_start, self.was_finish = False, False
        
        # Блокировки  
        self.fruit_lock = Lock()
        self.throw_short_lock = Lock()
        self.init_lock = Lock()
        self.count_blocks_lock = Lock()

        play_audio('CALGON.wav')
        
        self.main_thread = ManagedThread(
            name="main",
            node=self,
            target_function=self.main_loop
        )

        write_log(f"\n\n\n{get_time()} Start..")
        

    def friut_callback(self, msg):
        with self.fruit_lock:
                self.fruit_classif = json.loads(msg.data)

    def throw_short_callback(self, msg):
        self.throw_short_status = msg.data

    def init_callback(self, msg):
        with self.init_lock:
            self.init_command = msg.data
            print(self.init_command)

    
    def send_start_finish(self, number):
        print(number)
        assert isinstance(number, int)
        msg = Int8(data = number)
        self.start_finish_publ.publish(msg)

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

##################

    def main_loop(self, thread):

        while True:
                with self.init_lock:
                    if self.init_command == 1:
                        break
                    
        while thread.running and rclpy.ok():

            if not self.was_start:
                thread.start_child(target_function=lambda t: self.start_move(t))
                self.was_start = True

            with self.init_lock:
                if self.init_command == 1:
                    self.send_border_mode(1)
                elif self.init_command == -1:
                    self.send_border_mode(0)

            with self.fruit_lock:
                if self.fruit_classif:
                    if self.fruit_classif['x'] > 0.48 and self.fruit_classif['class'] != self.last_fruit:
                        self.send_border_mode(0)
                        write_log(f"\n{get_time()} Detect {self.fruit_classif['class']} !!!!!!! ")
                        self.count_blocks += 1
                        self.last_fruit = self.fruit_classif['class']
                        thread.start_child(
                            target_function=lambda t: self.process_fruit(t)
                        )
                        #thread.paused.wait()

            with self.count_blocks_lock:
                if self.count_blocks == 6:
                    if not self.was_finish:
                        thread.start_child(target_function=lambda t: self.finish_move(t))
                        #thread.paused.wait()
                        self.was_finish = True
                        thread.stop()
                        break

            with self.init_lock:
                if self.init_command == -1:
                    msg = Twist()
                    for i in range(10):
                        self.send_border_mode(0)
                        self.emergency_stop_publ.publish(msg)
                    thread.stop()
                    break
                    

    def process_fruit(self, thread):
    
        fruit_type = self.fruit_classif.get('class', 'unknown')
        play_audio(f'{fruit_type}.wav')
        action = conf.matching.get(fruit_type, 'unknown')

        write_log(f"\n{get_time()} Action {action} !!! ")
        
        if action == 'knock_down':
            self.knock_down_fruit(thread)
        elif action == 'pick':
            self.pick_up_fruit(thread)
        else:
            self.ignore_fruit(thread)


    def knock_down_fruit(self, thread):
        write_log(log_string = f"\n{get_time()} Start knock down !!! ")
        self.send_arm_action('knock_down')
        write_log(f"\n{get_time()} start sleep.. knock_down")
        time.sleep(conf.st_delay * 12)
        write_log(f"\n{get_time()} finish sleep.. knock_down")


    def pick_up_fruit(self, thread):
        write_log(log_string = f"\n{get_time()} Start pick up !!! ")
        self.send_arm_action('pick')
        write_log(log_string = f"\n{get_time()} start sleep.. pick")
        time.sleep(conf.st_delay * 20)
        write_log(f"\n{get_time()} finish sleep.. pick")

        current_block = self.count_blocks
        write_log(f"\n{get_time()} Start dual.. pick")
        thread.start_child(target_function=lambda t: self.maneuver(t, [1, current_block]))

    def throw_fruit(self, thread):
        write_log(f"\n{get_time()} Start throw !!! ")
        self.send_arm_action('throw_short_side')

        write_log(f"\n{get_time()} start sleep.. throw")
        time.sleep(conf.st_delay * 33)
        write_log(f"\n{get_time()} finish sleep.. throw")

    def ignore_fruit(self, thread):
        write_log(f"\n{get_time()} Ignoring !!! ")
        time.sleep(3.0)
        write_log(f"\n{get_time()} finish Ignoring..")

    def maneuver(self, thread, data):

        write_log(f"\n{get_time()} Start maneuver {data} !!! ")
        self.send_throw_short_mode(data)
        block = data[1]

        write_log(f"\n{get_time()} start sleep.. maneuver")
        sleep_time = 30.0 if block in [1,3,4,6] else 20.0
        time.sleep(sleep_time)
        write_log(f"\n{get_time()} finish sleep.. maneuver")

        thread.start_child(target_function=lambda t: self.throw_fruit(t))
        

    def start_move(self, thread):
        write_log(f"\n{get_time()} Start move start !!! ")
        self.send_start_finish(1)
        write_log(f"\n{get_time()} start sleep.. start")
        time.sleep(5.0)
        write_log(f"\n{get_time()} finish sleep.. start")

    def finish_move(self, thread):
        write_log(f"\n{get_time()} Start move finish !!! ")
        self.send_start_finish(-1)
        write_log(f"\n{get_time()} start sleep.. finish")
        time.sleep(9.0)
        write_log(f"\n{get_time()} finish sleep.. finish")

##########################     


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    node.main_thread.start()
    rclpy.spin(node)
    node.main_thread.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
