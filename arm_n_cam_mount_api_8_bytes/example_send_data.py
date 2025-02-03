import serial
import arm_n_cam_mount_api_8_bytes.send_data as send_data

port = "COM5"
baud_rate = 115200
ser = serial.Serial(port, baud_rate, timeout=1)    

# try:
#     ser.open()
# except:
#     print(f'Failed to open {port} port!')
#     exit(-42)

while True:
    angles = [130, 140, 120, 145, 125, 130, 150] # Дефолтные значения

    input_ang = input() # Ожидается не более 6 чисел, дфеолтные соответствующие порядку числа будут заменены на введённые
    if input_ang == 'f':
        break
    input_ang = input_ang.split(' ')
    for i, el in enumerate(input_ang):
        if i > 6:
            print("More than 6")
            break
        try:
            angles[i] = int(el)
        except:
            print("Exc")

    send_data.send_to_serial(ser, angles)

ser.close()