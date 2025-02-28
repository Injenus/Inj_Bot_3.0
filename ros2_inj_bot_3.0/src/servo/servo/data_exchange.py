import sys
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial

current_script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(current_script_path)
receive_data_path = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', '..', '..', '..', 'arm_n_cam_mount_api_8_bytes'))
if receive_data_path not in sys.path:
    sys.path.append(receive_data_path)

import receive_data
import send_data

# def fletcher8(data):
#     """Рассчитывает контрольную сумму по методу Флетчера-8 (результат - 1 байт)"""
#     sum1 = 0
#     sum2 = 0
#     for byte in data:
#         sum1 = (sum1 + byte) % 255
#         sum2 = (sum2 + sum1) % 255
#     return (sum1 + sum2) % 255

# def create_byte_array(angles):
#     """Создаёт массив байтов в формате <A><angle0><angle1>...<angle6><SUM>"""
#     if len(angles) != 7:
#         raise ValueError("Должно быть ровно 7 углов.")
#     data = bytearray(b'A')
#     for angle in angles:
#         if not (0 <= angle <= 255):
#             raise ValueError("Значения углов должны быть в диапазоне от 0 до 255.")
#         data.append(angle)
#     checksum = fletcher8(data)
#     data.append(checksum)
#     return data

# def send_to_serial(ser, angles):
#     """Отправляет данные по указанному COM-порту (без изменений)"""
#     data = create_byte_array(angles)
#     print(f"Отправляемые данные {angles}: {data}")
#     ser.write(data)
#     ser.flush()

class WriteReadServoData(Node):
    def __init__(self):
        super().__init__('wr_servo_data')
        self.publisher = self.create_publisher(UInt8MultiArray, 'servo/current', 1)
        self.serial_port = None
        self.running = False
        self.serial_lock = threading.Lock()  # Блокировка для потокобезопасности

        # Подписчик для команд отправки
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'servo/to_write',
            self.command_callback,
            1
        )

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

    def command_callback(self, msg):
        """Обработка входящих команд для отправки"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Порт не открыт")
            return

        if len(msg.data) != 7:
            self.get_logger().error("Требуется 7 значений углов")
            return

        try:
            with self.serial_lock:  # Захватываем блокировку
                send_data.send_to_serial(self.serial_port, msg.data)
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки: {e}")

    def read_from_port(self):
        buffer = bytearray()
        START_BYTE = ord('A')
        PACKET_LENGTH = 9

        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                # Чтение строго по одному байту (оригинальная логика)
                with self.serial_lock:  # Захватываем блокировку только на время чтения
                    while self.serial_port.in_waiting > 0:
                        byte = self.serial_port.read(1)
                        buffer.append(byte[0])
            
                # Обработка буфера ВНЕ блокировки
                if len(buffer) >= PACKET_LENGTH:
                    start_index = buffer.find(bytes([START_BYTE]))
                    if start_index != -1:
                        if len(buffer) - start_index >= PACKET_LENGTH:
                            packet = buffer[start_index:start_index + PACKET_LENGTH]
                            parsed_data = receive_data.parse_data(packet)
                            
                            if parsed_data:
                                msg = UInt8MultiArray()
                                msg.data = parsed_data[1:]
                                self.publisher.publish(msg)
                                self.get_logger().info(f"Published: {parsed_data[1:]}")

                            buffer = buffer[start_index + PACKET_LENGTH:]
                        else:
                            break
                    else:
                        buffer = buffer[-1:]

            except Exception as e:
                self.get_logger().error(f"Ошибка: {e}")
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
    node = WriteReadServoData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()