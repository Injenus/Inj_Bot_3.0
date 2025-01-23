import struct
import serial

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

def send_to_serial(port, angles):
    """Отправляет данные по указанному COM-порту"""
    data = create_byte_array(angles)
    with serial.Serial(port, baudrate=115200, timeout=1) as ser:
        ser.write(data)
        print(f"Данные отправлены: {data}")

if __name__ == "__main__":
    com_port = 'COM5'            
    angles = [130, 140, 120, 145, 125, 130, 150]
    
    while 1:
        send_to_serial(com_port, angles)
