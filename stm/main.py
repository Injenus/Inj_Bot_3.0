import serial
import struct

# Настройки последовательного порта
SERIAL_PORT = "/dev/ttyS0"  # Укажите правильный порт для вашего Raspberry Pi
BAUD_RATE = 115200

# Функция для вычисления Fletcher-16
def fletcher16(data):
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1

# Функция для обработки данных
def process_message(data):
    # Проверяем длину сообщения
    if len(data) != 19:
        print("Ошибка: некорректная длина сообщения")
        return None

    # Проверяем стартовый символ
    if data[0] != ord('S'):
        print("Ошибка: неверный стартовый символ")
        return None

    # Извлекаем контрольную сумму
    received_checksum = struct.unpack('H', data[17:19])[0]

    # Проверяем контрольную сумму
    if fletcher16(data[:17]) != received_checksum:
        print("Ошибка: контрольная сумма не совпадает")
        return None

    # Извлекаем скорости (4 float значения по 4 байта)
    speeds = struct.unpack('ffff', data[1:17])
    return speeds

# Открываем последовательный порт
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Подключено к {SERIAL_PORT} со скоростью {BAUD_RATE} бод")
except serial.SerialException as e:
    print(f"Не удалось открыть порт: {e}")
    exit()

# Основной цикл с синхронизацией
print("Ожидание данных...")
buffer = bytearray()  # Буфер для хранения входящих данных

while True:
    try:
        # Читаем байты из порта
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))

        # Проверяем, есть ли в буфере стартовый символ
        while len(buffer) >= 19:
            # Ищем стартовый символ
            if buffer[0] != ord('S'):
                buffer.pop(0)  # Удаляем лишние данные до символа 'S'
                continue

            # Если в буфере меньше 19 байт, ждём следующих данных
            if len(buffer) < 19:
                break

            # Извлекаем потенциальное сообщение
            message = buffer[:19]
            buffer = buffer[19:]  # Удаляем обработанное сообщение из буфера

            # Обрабатываем сообщение
            speeds = process_message(message)
            if speeds:
                print(f"Скорости (об/мин): {speeds}")
    except KeyboardInterrupt:
        print("\nЗавершение работы")
        break
    except Exception as e:
        print(f"Ошибка: {e}")
