import serial

def fletcher8_checksum(data):
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    #print(list(data), (sum1 + sum2) % 255)
    return (sum1 + sum2) % 255


def parse_data(packet):
    if len(packet) != 9:
        return None
    if packet[0] != ord('A'):
        return None
    data = packet[:8]
    checksum = packet[8]
    if fletcher8_checksum(data) != checksum:
        return None
    return list(data)[1:]

def main():
    START_BYTE = ord('A')
    PACKET_LENGTH = 9 
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    buffer = bytearray()

    while True:
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
                        parsed_data = parse_data(packet)

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

if __name__ == "__main__":
    main()
