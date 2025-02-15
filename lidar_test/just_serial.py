import serial
import time

# Настройки последовательного порта
port = 'COM7'  # Замените на ваш порт
baudrate = 460800

# Открытие порта
ser = serial.Serial(port, baudrate, timeout=1)

# Команды управления
start_scan = b'\xA5\x50'
stop_scan = b'\xA5\x65'

try:
    # Отправка команды запуска
    ser.write(start_scan)
    time.sleep(0.1)  # Небольшая задержка

    # Сбор данных в течение 5 секунд
    start_time = time.time()
    with open('lidar_data.txt', 'wb') as f:
        while time.time() - start_time < 5:
            data = ser.read(1024)  # Чтение данных
            if data:
                f.write(data)

    # Отправка команды остановки
    ser.write(stop_scan)

finally:
    # Закрытие порта
    ser.close()
