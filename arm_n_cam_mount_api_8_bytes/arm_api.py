def fletcher8(data):
    """Рассчитывает контрольную сумму по методу Флетчера-8 (результат - 1 байт)"""
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum1 + sum2) % 255  # Возвращаем результат как 1 байт

def create_byte_array(angles):
    """Создаёт массив байтов в формате <A><angle0><angle1>...<angle6><SUM>"""
    if len(angles) != 7:
        raise ValueError("Должно быть ровно 7 углов.")

    # Инициализируем массив байтов с символа 'A'
    data = bytearray(b'A')

    # Добавляем углы (каждое значение занимает 1 байт)
    for angle in angles:
        if not (0 <= angle <= 255):
            raise ValueError("Значения углов должны быть в диапазоне от 0 до 255.")
        data.append(angle)

    checksum = fletcher8(data)
    data.append(checksum)

    return data

def send_to_serial(ser, angles):
    """Отправляет данные по указанному COM-порту"""
    data = create_byte_array(angles)
    print(f"Отправляемые данные {angles}: {data}")
    ser.write(data)  # Отправляем все байты разом
    ser.flush()
