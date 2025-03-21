import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial
import copy
import time

current_script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(current_script_path)
# print(script_dir)
receive_data_path = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..', '..', '..', 'arm_n_cam_mount_api_8_bytes'))
# print(receive_data_path)
if receive_data_path not in sys.path:
    sys.path.append(receive_data_path)

import receive_data
import send_data


class ServoSerialReadWrite(Node):
    def __init__(self):
        super().__init__('servo_read_write')
        self.receive_data_publisher = self.create_publisher(UInt8MultiArray, 'servo/current', 3)
        self.serial_port = None

        self.subscription = self.create_subscription(UInt8MultiArray, 'servo/to_write', self.command_callback, 3)
        
        self.last_data = None
        self.send_attempts = 0

        try:
            self.serial_port = serial.Serial(
                port='/dev/SERVOs',
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        self.create_timer(0.040, self.main_loop)
        

        servo_msg = UInt8MultiArray()
        servo_msg.data = bytes([142-90, 133+60, 130+30, 132, 0, 128, 128])
        send_data.send_to_serial(self.serial_port, servo_msg.data)

    
    def command_callback(self, msg):

        if self.last_data != msg.data:
            self.last_data = copy.deepcopy(msg.data)
            self.send_attempts = 3


    def main_loop(self):

        # отправляем целевые данные на драйвер - из топика ожидаюся углы уже в формате uint8_t
        while self.send_attempts > 0:
            send_data.send_to_serial(self.serial_port, self.last_data)
            self.send_attempts -= 1
            print(f'Send to driver {self.last_data}')

        start_time = time.time()
        max_runtime = 0.025
        intime = True
        rec_buff = bytearray()
        PACKET_LENGTH = 9
        START_BYTE = ord('A')
        # приём данных (идут регулярно)
        while intime:
            if self.serial_port.in_waiting > 0:
                byte = self.serial_port.read(1)
                rec_buff.append(byte[0])
                # принятые баты

            if len(rec_buff) >= PACKET_LENGTH:
                start_idx = rec_buff.find(bytes([START_BYTE]))
                if start_idx != -1:
                    if len(rec_buff) - start_idx >= PACKET_LENGTH:
                        packet = rec_buff[start_idx:start_idx+PACKET_LENGTH]
                        parsed_data = receive_data.parse_data(packet)[1:] # убираем первый байт-символ

                        if parsed_data:
                            #elf.get_logger().info(f'Got data: {parsed_data}')  # в таком виде имеем список из 7 
                            break # выходим из цикла поулчания байт - ном притшло корреткное сообщение о состоянии
                        else:
                            self.get_logger().info("Receive err")

                        rec_buff = rec_buff[start_idx + PACKET_LENGTH]
                else:
                    rec_buff = rec_buff[-1:]

            if time.time() - start_time > max_runtime:
                print(time.time() - start_time, max_runtime)
                intime = False

        #получили данные, отпарвляем в топик текущих положений
        if intime:
            msg = UInt8MultiArray()
            msg.data = parsed_data
            self.receive_data_publisher.publish(msg)
            #self.get_logger().info(f'Publ data: {msg.data}')
        else:
            self.get_logger().info('Nothing to publ., timeout')


    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    servo_data_exchange = ServoSerialReadWrite()
    # try:
    #     rclpy.spin(servo_data_exchange)
    # except:
    #     servo_data_exchange.destroy_node()
    #     rclpy.shutdown()

    rclpy.spin(servo_data_exchange)

if __name__ == '__main__':
    main()