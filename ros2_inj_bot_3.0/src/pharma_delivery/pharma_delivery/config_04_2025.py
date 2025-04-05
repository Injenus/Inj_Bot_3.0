# Параметры и константы


LIDAR_STEP = 10

LIN_X_SPEED = 0.15 # m/s
LIN_Y_SPEED = 0.1
ANG_Z_SPEED = 2.0 # rad/s
P_koef = 10.0

thresh_taking_pharma_err = 0.015
thresh_lidar_move = 0.003
thresh_lidar_pharma = 0.002

offset_0 = -5
arm_positions = {
    0 : ([142-offset_0, 133, 130, 132, 0, 128, 42], 'init'),
    1 : ([142-offset_0-90, 133+75, 130-120, 132, 0, 128, 42],'search_qr'),
    2 : ([142-offset_0-45, 133+38, 130-60, 132, 0, 128, 42], 'interim_qr_direct'),
    3 : ([142-offset_0, 133+75, 130-120, 132, 0, 128, 42], 'direct_cam'),

    11 : ([142-offset_0-90-25, 133+75, 130-120, 132, 0, 128, 42], 'search_qr_shake_left'),
    111 : ([142-offset_0-90+25, 133+75, 130-120, 0, 128, 42], 'search_qr_shake_right'),

    33 : ([142-10-25, 133+75, 130-120, 132, 0, 128, 42], 'left_shaking_direct_cam'),
    333 : ([142-10+25, 133+75, 130-120, 132, 0, 128, 42], 'right_shaking_direct_cam')
}

# рассторния от центра лидара до соотв. краёв робота (метры)
lidar_radiues = 0.0507/2
lidar_offsets = {
    'front': 0.010+lidar_radiues,
    'left': 0.100+lidar_radiues,
    'right': 0.100+lidar_radiues,
    'back': 0.260+lidar_radiues,
    'right_one_cell': 0.100+lidar_radiues+0.8
}

# минимальные расстония до стенки лабиринта
wall_distance = 0.0442
pharma_close_distance = lidar_offsets['front'] + 0.03
pharma_for_back_dictance = lidar_offsets['back']

confirm_time = 2.0 # sec - для всяких остановок 