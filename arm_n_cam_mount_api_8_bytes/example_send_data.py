import serial
import json

import arm_n_cam_mount_api_8_bytes.send_data as send_data

port = "COM5"
baud_rate = 115200
ser = serial.Serial(port, baud_rate, timeout=1)    

# try:
#     ser.open()
# except:
#     print(f'Failed to open {port} port!')
#     exit(-42)

with open('config_arm.json', 'r') as file:
        config = json.load(file)
        arm_offet = config['offset']

with open('config_cam.json', 'r') as file:
        config = json.load(file)
        cam_offet = config['offset']

offset = arm_offet + cam_offet
while True:
    user_angles = [45, 10, 0, -100, 1, 0, 0] # Дефолтные
    
    input_ang = input() # Ожидается не более 6 чисел, дфеолтные соответствующие порядку числа будут заменены на введённые
    if input_ang == 'f':
        break
    input_ang = input_ang.split(' ')
    for i, el in enumerate(input_ang):
        if i > 6:
            print("More than 6")
            break
        try:
            ang = int(el)
            if -128 <= ang <= 127:
                user_angles[i] = ang
            else:
                 print(f"Угол {i} не был заменён: выход за пределы диапазона")
        except:
            print("Exc")

    angles = [ang + offset[i] for i, ang in enumerate(user_angles)]
    send_data.send_to_serial(ser, angles)

ser.close()