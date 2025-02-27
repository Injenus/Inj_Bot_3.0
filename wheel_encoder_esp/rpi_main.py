import serial
import struct

SERIAL_PORT = "/dev/ttyUSB0"  # Пример для Linux, для Windows используйте "COMx"
BAUD_RATE = 115200

def fletcher16(data):
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Port error: {e}")
    exit()

buffer = bytearray()
STATE_SEARCH = 0
STATE_READ_DATA = 1
state = STATE_SEARCH
expected_length = 19  # 1 (S) + 16 (4 float) + 2 (checksum)

while True:
    try:
        # Чтение данных порциями
        data = ser.read(1)
        if data:
            buffer.extend(data)
        
        # Обработка буфера
        while True:
            if state == STATE_SEARCH:
                # Поиск стартового байта 'S' (0x53)
                start_pos = buffer.find(b'S')
                if start_pos == -1:
                    buffer.clear()  # Нет 'S' - очистка буфера
                    break
                
                # Переходим в режим чтения данных
                buffer = buffer[start_pos:]  # Обрезаем все до 'S'
                state = STATE_READ_DATA

            elif state == STATE_READ_DATA:
                if len(buffer) < expected_length:
                    break  # Ждем больше данных
                
                # Извлекаем пакет
                packet = buffer[:expected_length]
                buffer = buffer[expected_length:]
                
                # Проверка контрольной суммы
                calc_csum = fletcher16(packet[:17])
                recv_csum = struct.unpack_from('<H', packet, 17)[0]
                
                if calc_csum != recv_csum:
                    print("Checksum error! Resyncing...")
                    state = STATE_SEARCH
                    continue
                
                # Распаковка данных (little-endian)
                speeds = struct.unpack_from('<4f', packet, 1)
                print(f"Speeds (FL, RL, FR, RR): {speeds}")
                
                state = STATE_SEARCH  # Возврат к поиску

    except KeyboardInterrupt:
        print("\nExiting...")
        break
    except Exception as e:
        print(f"Error: {e}")
        state = STATE_SEARCH  # Сброс состояния при ошибке

ser.close()