import time
from rplidar import RPLidar

# Укажите порт, к которому подключен ваш лидар (например, '/dev/ttyUSB0')
PORT_NAME = "/dev/ttyUSB0"

def check_lidar(frequency):
    """Функция включает лидар, записывает данные и выключает его."""
    lidar = RPLidar(PORT_NAME)
    
    # Установка частоты сканирования (обычно доступные: 5, 10, 15 Гц)
    lidar._set_pwm(int(frequency * 1000))  # Переводим в Гц и задаем в мГц

    print(f"Лидар включен. Запись данных на частоте {frequency} Гц...")

    with open("lidar_data.txt", "w") as f:
        start_time = time.time()
        for scan in lidar.iter_scans():
            for quality, angle, distance in scan:
                f.write(f"{time.time()-start_time:.3f}, {angle:.2f}, {distance:.2f}, {quality}\n")
            if time.time() - start_time > 5:
                break  # Останавливаем запись через 5 секунд
    
    print("Запись завершена. Лидар выключается...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == "__main__":
    freq = float(input("Введите частоту работы лидара (например, 5, 10, 15): "))
    check_lidar(freq)
