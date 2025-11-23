import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import serial
import serial.serialutil
import copy
import time

receive_data_path = os.path.join(os.path.expanduser('~'),
                                 'Inj_Bot_3.0',
                                 'arm_n_cam_mount_api_8_bytes')
if receive_data_path not in sys.path:
    sys.path.append(receive_data_path)

import receive_data
import send_data


class ServoSerialReadWrite(Node):
    def __init__(self):
        super().__init__('servo_read_write')

        # Паблишеры / подписки
        self.receive_data_publisher = self.create_publisher(
            UInt8MultiArray, 'servo/current', 3
        )
        self.subscription = self.create_subscription(
            UInt8MultiArray, 'servo/to_write', self.command_callback, 3
        )
        self.ready_pub = self.create_publisher(String, '/nodes_ready', 10)

        # Состояние
        self.serial_port = None
        self.last_data = None
        self.send_attempts = 0

        # Пытаемся открыть порт один раз при старте
        self.open_serial()

        # Таймер основного цикла
        self.create_timer(0.040, self.main_loop)

        # Сообщаем координатору, что узел запущен (это не значит, что порт жив)
        msg = String(data=self.get_name())
        self.ready_pub.publish(msg)

    # ---------- Работа с serial ----------

    def open_serial(self):
        """Попытка открыть последовательный порт."""
        try:
            # Здесь можно использовать /dev/serial/by-id/... вместо /dev/SERVOs
            self.serial_port = serial.Serial(
                port='/dev/SERVOs',
                baudrate=115200,
                timeout=0.01  # мало-блокирующий режим
            )
            self.get_logger().info("Serial port /dev/SERVOs opened successfully")
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Failed to open serial port /dev/SERVOs: {e}")
            self.serial_port = None

    def close_serial(self):
        """Безопасно закрыть порт."""
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
                    self.get_logger().info("Serial port /dev/SERVOs closed")
            except Exception as e:
                self.get_logger().error(f"Error while closing serial port: {e}")
        self.serial_port = None

    # ---------- ROS callbacks ----------

    def command_callback(self, msg: UInt8MultiArray):
        """Получаем целевые данные для сервоприводов."""
        if self.last_data != msg.data:
            # Копируем, чтобы не ссылаться на внутренний буфер ROS
            self.last_data = copy.deepcopy(msg.data)
            self.send_attempts = 3  # отправим несколько раз для надёжности

    def main_loop(self):
        """Основной цикл: отправка команд и приём текущих положений."""

        # Если порта нет — пробуем открыть и выходим до следующего такта
        if self.serial_port is None:
            self.open_serial()
            return

        try:
            # ---------- Отправка команд ----------
            if self.last_data is not None and self.send_attempts > 0:
                while self.send_attempts > 0:
                    send_data.send_to_serial(self.serial_port, self.last_data)
                    self.send_attempts -= 1
                    # self.get_logger().debug(f'Send to driver {self.last_data}')

            # ---------- Приём данных ----------
            start_time = time.time()
            max_runtime = 0.025
            intime = True
            rec_buff = bytearray()
            PACKET_LENGTH = 9
            START_BYTE = ord('A')
            parsed_data = None

            while intime:
                # Проверяем, пришли ли данные
                if self.serial_port.in_waiting > 0:
                    byte = self.serial_port.read(1)
                    if not byte:
                        # Ничего не прочитали, выходим по таймауту
                        pass
                    else:
                        rec_buff.append(byte[0])

                # Пытаемся вытащить пакет
                if len(rec_buff) >= PACKET_LENGTH:
                    start_idx = rec_buff.find(bytes([START_BYTE]))
                    if start_idx != -1:
                        if len(rec_buff) - start_idx >= PACKET_LENGTH:
                            packet = rec_buff[start_idx:start_idx + PACKET_LENGTH]
                            parsed_data = receive_data.parse_data(packet)
                            if parsed_data:
                                # Корректный пакет — выходим из цикла приёма
                                break
                            else:
                                self.get_logger().info("Receive err (parse failed)")
                            # удаляем использованный кусок
                            rec_buff = rec_buff[start_idx + PACKET_LENGTH:]
                    else:
                        # нет стартового байта — оставляем только последний
                        rec_buff = rec_buff[-1:]

                # Контроль времени
                if time.time() - start_time > max_runtime:
                    intime = False

            # ---------- Публикация ----------
            if intime and parsed_data:
                msg = UInt8MultiArray()
                msg.data = parsed_data
                self.receive_data_publisher.publish(msg)
                # self.get_logger().debug(f'Publ data: {msg.data}')
            else:
                # self.get_logger().info('Nothing to publ., timeout or no data')
                pass

        except (serial.serialutil.SerialException, OSError) as e:
            # Любая ошибка работы с портом → закрываем и будем переподключаться
            self.get_logger().error(
                f"Serial error on /dev/SERVOs: {e}. Resetting port and will retry..."
            )
            self.close_serial()
        except Exception as e:
            # Непредвиденная ошибка — логируем, но не даём таймеру умереть
            self.get_logger().error(f"Unexpected error in main_loop: {e}")

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    servo_data_exchange = ServoSerialReadWrite()
    try:
        rclpy.spin(servo_data_exchange)
    finally:
        servo_data_exchange.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
