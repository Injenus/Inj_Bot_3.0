from rplidar import RPLidar

PORT = "COM7"  # Или /dev/ttyUSB0 на Raspberry Pi

lidar = RPLidar(PORT)

info = lidar.get_info()
print("Информация о лидаре:", info)

health = lidar.get_health()
print("Состояние лидара:", health)

lidar.stop()
lidar.disconnect()
