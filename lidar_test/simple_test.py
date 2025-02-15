import serial

PORT = "COM7"  # Укажи свой порт (или /dev/ttyUSB0 для Linux)
BAUDRATE = 460800  # Попробуй 256000, 512000, если не работает

try:
    with serial.Serial(PORT, BAUDRATE, timeout=2) as ser:
        ser.write(b'\xA5\x50')  # Запросить информацию о лидара
        response = ser.read(100)  # Считать ответ
        print("Ответ лидара:", response)
except Exception as e:
    print("Ошибка:", e)
