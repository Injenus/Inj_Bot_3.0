import sys
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial

current_script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(current_script_path)
# print(script_dir)
receive_data_path = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..', '..', '..', 'arm_n_cam_mount_api_8_bytes'))
# print(receive_data_path)
if receive_data_path not in sys.path:
    sys.path.append(receive_data_path)

import receive_data


class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_arm_reader')
        self.publisher = self.create_publisher(UInt8MultiArray, 'servo/current', 1)
        self.serial_port = None
        self.running = False
        
        try:
            self.serial_port = serial.Serial(
                port='/dev/servo_arduino',
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info("Serial port opened successfully")
            self.running = True
            self.read_thread = threading.Thread(target=self.read_from_port)
            self.read_thread.start()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

    def read_from_port(self):
        buffer = bytearray()
        START_BYTE = ord('A')
        PACKET_LENGTH = 9
        

        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                while self.serial_port.in_waiting:
                    byte = self.serial_port.read(1)
                    buffer.append(byte[0])  # Добавляем байт в буфер
                    #print("Буфер:", list(buffer))

                    # Проверяем наличие стартового байта и достаточной длины
                    if len(buffer) >= PACKET_LENGTH:
                        # Поиск первого стартового байта
                        start_index = buffer.find(bytes([START_BYTE]))
                        if start_index != -1:
                            # Проверяем, есть ли полный пакет после стартового байта
                            if len(buffer) - start_index >= PACKET_LENGTH:
                                packet = buffer[start_index:start_index + PACKET_LENGTH]
                                parsed_data = receive_data.parse_data(packet)

                                if parsed_data:
                                    # Создаем и публикуем сообщение
                                    msg = UInt8MultiArray()
                                    msg.data = parsed_data[1:]  # Исключаем стартовый байт
                                    self.publisher.publish(msg)
                                    self.get_logger().info(f"Published data: {parsed_data[1:]}") #, throttle_duration_sec=1)
                                else:
                                    self.get_logger().info("Ошибка контрольной суммы или неверный пакет")

                                # Удаляем обработанный пакет из буфера
                                buffer = buffer[start_index + PACKET_LENGTH:]
                            else:
                                # Ждем дополнительных байтов
                                break
                        else:
                            # Удаляем "мусор" до первого найденного стартового байта
                            buffer = buffer[-1:]
                            
            except Exception as e:
                self.get_logger().info(f"Error reading from port: {e}")
                break

    def destroy_node(self):
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    try:
        rclpy.spin(node)
    except:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()