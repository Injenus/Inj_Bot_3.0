import struct

def fletcher16(data):
    """Рассчитывает контрольную сумму по методу Флетчера-16"""
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1

def create_byte_array(angles):
    """Создаёт массив байтов в формате <A><angle0><angle0>...<angle6><angle6><SUM><SUM>"""
    if len(angles) != 7:
        raise ValueError("Должно быть ровно 7 углов.")
    
    data = bytearray(b'A')
    
    # Добавляем каждый угол дважды (в формате little-endian)
    for angle in angles:
        data += struct.pack('<h', angle) * 2  # <h - 2-байтовый int, little-endian
    
    checksum = fletcher16(data)
    data += struct.pack('<H', checksum)  # <H - 2-байтовое unsigned int, little-endian
    
    return data

def send_to_serial(ser, angles):
    """Отправляет данные по указанному COM-порту"""
    data = create_byte_array(angles)
    #print(f"Отправляемые данные: {data}")
    ser.write(data)
    ser.flush()  # Очистка буфера для отправки
    #print(f"Receive: {ser.readline()}")



