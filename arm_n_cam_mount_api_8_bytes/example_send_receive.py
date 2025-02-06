import serial
import keyboard
from multiprocessing import Process
import random
import time

import receive_data
import send_data



def main():
    ser = serial.Serial('COM5', 115200, timeout=1)
    user_angles = [45, 10, 0, -100, 1, 0, 0] # Дефолтные
    START_BYTE = ord('A')
    PACKET_LENGTH = 9
    buffer = bytearray()
    timer = 0
    period = 1.5
    ang_range = 45

    while True:
        if keyboard.is_pressed('e'):
            break

        while ser.in_waiting:
            byte = ser.read(1)
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
                            print("Полученные данные:", parsed_data[1:])
                        else:
                            print("Ошибка контрольной суммы или неверный пакет")

                        # Удаляем обработанный пакет из буфера
                        buffer = buffer[start_index + PACKET_LENGTH:]
                    else:
                        # Ждем дополнительных байтов
                        break
                else:
                    # Удаляем "мусор" до первого найденного стартового байта
                    buffer = buffer[-1:]

        if time.time() - timer > period:
            timer = time.time() 
            user_angles = [random.randint(-ang_range,ang_range),random.randint(-ang_range,ang_range), random.randint(-ang_range,ang_range), random.randint(-ang_range,ang_range), random.randint(0,1), random.randint(-87,87), random.randint(-90, 30)]
            angles = [ang + 128 if i <= 3 else (ang if i == 4 else (ang + 87 if i==5 else ang + 90)) for i, ang in enumerate(user_angles)]
            send_data.send_to_serial(ser, angles)


if __name__ == "__main__":
    main_p= Process(target=main, args=())
    main_p.start()
    main_p.join()