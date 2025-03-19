import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import copy
import time

current_script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(current_script_path)
# print(script_dir)
receive_data_path = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..', '..', '..', 'wheel_encoder_esp'))
# print(receive_data_path)
if receive_data_path not in sys.path:
    sys.path.append(receive_data_path)

import wheel_modules

class WheelSerialReadWrite(Node):
    def __init__(self):
        super().__init__('wheel_read_write')
        self.receive_data_publisher = self.create_publisher(Int32MultiArray,'wheel/current_rpm', 3)
        self.serial_port = None

        self.subscription = self.create_subscription(Int32MultiArray, 'wheel/target_rpm', self.command_callback, 3)
        
        self.last_data = None
        self.send_attempts = 0

        try:
            self.serial_port = serial.Serial(
                port = '/dev/ttyUSB2',
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        self.create_timer(0.005, self.main_loop)

    def command_callback(self, msg):
          if self.last_data != msg.data:
              self.last_data = copy.deepcopy(msg.data)
              self.send_attempts = 3


    def main_loop(self):
        while self.send_attempts > 0:
            wheel_modules.send_to_serial(self.serial_port, self.last_data)
            self.send_attempts -= 1

        start_time = time.time()
        max_runtime = 0.025
        intime = True
        rec_buff = bytearray()
        PACKET_LENGTH = 19
        START_BYTE = ord('S')

        while intime:
            byte = self.serial_port.read(1)
            rec_buff.append(byte[0])

            # Поиск и обработка полных пакетов
            while True:
                # Поиск стартового байта
                start_pos = rec_buff.find(START_BYTE.to_bytes(1, 'big'))
                
                if start_pos == -1:
                    # Нет стартового байта - очищаем буфер
                    rec_buff.clear()
                    break
                
                if start_pos > 0:
                    # Удаляем мусор до стартового байта
                    del rec_buff[:start_pos]
                
                if len(rec_buff) < PACKET_LENGTH:
                    # Не хватает данных для полного пакета
                    break
                
                # Извлекаем потенциальный пакет
                packet = rec_buff[:PACKET_LENGTH]
                del rec_buff[:PACKET_LENGTH]
                
                parsed = wheel_modules.parse_packet(packet)
                if parsed is not None:
                    print("Received RPMs:", parsed)
                else:
                    print("Invalid packet received")

            if time.time() - start_time > max_runtime:
                print(time.time() - start_time, max_runtime)
                intime = False

        if intime:
            msg = Int32MultiArray()
            msg.data = parsed
            self.receive_data_publisher.publish(msg)
            self.get_logger().info(f'Publ data: {msg.data}')
        else:
            self.get_logger().info('Nothing to publ., timeout')


    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    wheel_data_exchange = WheelSerialReadWrite()
    # try:
    #     rclpy.spin(servo_data_exchange)
    # except:
    #     servo_data_exchange.destroy_node()
    #     rclpy.shutdown()

    rclpy.spin(wheel_data_exchange)

if __name__ == '__main__':
    main()