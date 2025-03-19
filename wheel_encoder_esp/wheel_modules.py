import serial
import struct

# SERIAL_PORT = "/dev/ttyUSB0"  # Пример для Linux, для Windows используйте "COMx"
# BAUD_RATE = 115200

# def fletcher16(data):
#     sum1 = 0
#     sum2 = 0
#     for byte in data:
#         sum1 = (sum1 + byte) % 255
#         sum2 = (sum2 + sum1) % 255
#     return (sum2 << 8) | sum1

# try:
#     ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
#     print(f"Connected to {SERIAL_PORT}")
# except Exception as e:
#     print(f"Port error: {e}")
#     exit()

# buffer = bytearray()
# STATE_SEARCH = 0
# STATE_READ_DATA = 1
# state = STATE_SEARCH
# expected_length = 19  # 1 (S) + 16 (4 float) + 2 (checksum)

# while True:
#     try:
#         # Чтение данных порциями
#         data = ser.read(1)
#         if data:
#             buffer.extend(data)
        
#         # Обработка буфера
#         while True:
#             if state == STATE_SEARCH:
#                 # Поиск стартового байта 'S' (0x53)
#                 start_pos = buffer.find(b'S')
#                 if start_pos == -1:
#                     buffer.clear()  # Нет 'S' - очистка буфера
#                     break
                
#                 # Переходим в режим чтения данных
#                 buffer = buffer[start_pos:]  # Обрезаем все до 'S'
#                 state = STATE_READ_DATA

#             elif state == STATE_READ_DATA:
#                 if len(buffer) < expected_length:
#                     break  # Ждем больше данных
                
#                 # Извлекаем пакет
#                 packet = buffer[:expected_length]
#                 buffer = buffer[expected_length:]
                
#                 # Проверка контрольной суммы
#                 calc_csum = fletcher16(packet[:17])
#                 recv_csum = struct.unpack_from('<H', packet, 17)[0]
                
#                 if calc_csum != recv_csum:
#                     print("Checksum error! Resyncing...")
#                     state = STATE_SEARCH
#                     continue
                
#                 # Распаковка данных (little-endian)
#                 speeds = struct.unpack_from('<4f', packet, 1)
#                 print(f"Speeds (FL, RL, FR, RR): {speeds}")
                
#                 state = STATE_SEARCH  # Возврат к поиску

#     except KeyboardInterrupt:
#         print("\nExiting...")
#         break
#     except Exception as e:
#         print(f"Error: {e}")
#         state = STATE_SEARCH  # Сброс состояния при ошибке

# ser.close()

def fletcher16(data):
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1

def create_byte_array(speeds):
    if len(speeds) != 4:
        raise ValueError("Must be 4 speeds")
    
    data = bytearray(b'S')
    for speed in speeds:
        if not (-2147483648 <= speed <= 2147483647):
            raise ValueError("Must be int32 type")
        data.extend(speed.to_bytes(4, byteorder='little', signed=True))

    checksum = fletcher16(data)
    data.extend(checksum.to_bytes(2, byteorder='big'))

    return data

def send_to_serial(ser, speeds):
    data = create_byte_array(speeds)
    ser.write(data)
    ser.flush()


def fletcher16(data):
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1


def parse_packet(packet):
    if len(packet) != 19:
        return None
    if packet[0] != ord('S'):
        return None
    
    # Проверка контрольной суммы (первые 17 байт)
    received_checksum = (packet[17] << 8) | packet[18]
    calculated_checksum = fletcher16(packet[:17])
    
    if received_checksum != calculated_checksum:
        return None
    
    # Парсинг 4-байтовых значений (little-endian)
    rpms = []
    for i in range(4):
        start_idx = 1 + i*4
        value_bytes = packet[start_idx:start_idx+4]
        value = int.from_bytes(value_bytes, byteorder='little', signed=True)
        rpms.append(value)
    
    return rpms

def main_receive_example():
    START_BYTE = ord('S')
    PACKET_LENGTH = 19
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    buffer = bytearray()

    while True:
        # Чтение одного байта за итерацию
        while ser.in_waiting > 0:
            byte = ser.read(1)
            buffer.append(byte[0])

            # Поиск и обработка полных пакетов
            while True:
                # Поиск стартового байта
                start_pos = buffer.find(START_BYTE.to_bytes(1, 'big'))
                
                if start_pos == -1:
                    # Нет стартового байта - очищаем буфер
                    buffer.clear()
                    break
                
                if start_pos > 0:
                    # Удаляем мусор до стартового байта
                    del buffer[:start_pos]
                
                if len(buffer) < PACKET_LENGTH:
                    # Не хватает данных для полного пакета
                    break
                
                # Извлекаем потенциальный пакет
                packet = buffer[:PACKET_LENGTH]
                del buffer[:PACKET_LENGTH]
                
                # Парсим и проверяем
                parsed = parse_packet(packet)
                if parsed is not None:
                    print("Received RPMs:", parsed)
                else:
                    print("Invalid packet received")