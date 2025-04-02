# Параметры и константы

LIN_X_SPEED = 0.3 # m/s
LIN_Y_SPEED = 0.3
ANG_Z_SPEED = 1.5 # rad/s
P_koef = 1

thresh_taking_pharma_err = 0.05

arm_positions = {
    0 : ([142, 133, 130, 132, 0, 128, 42], 'init'),
    1 : ([142-90, 133+75, 130-120, 132, 0, 128, 42],'search_qr'),
    2 : ([142-45, 133+38, 130-60, 132, 0, 128, 42], 'interim_qr_direct'),
    3 : ([142, 133+75, 130-120, 132, 0, 128, 42], 'direct_cam')
}

# рассторния от центра лидара до соотв. краёв робота (метры)
lidar_radiues = 0.0507/2
lidar_offsets = {
    'front': 0.010+lidar_radiues,
    'left': 0.100+lidar_radiues,
    'right': 0.100+lidar_radiues,
    'back': 0.260+lidar_radiues
}

# минимальные расстония до стенки лабиринта
wall_distances = 0.100

confirm_time = 2.0 # sec - для всяких остановок 