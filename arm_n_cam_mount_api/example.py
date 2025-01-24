import serial
import arm_api

port = "COM5"  # Укажите ваш порт (на Windows это может быть COM3, COM4 и т.д.)
baud_rate = 115200
ser = serial.Serial(port, baud_rate, timeout=1)    

# try:
#     ser.open()
# except:
#     print(f'Failed to open {port} port!')
#     exit(-42)

while True:
    angles = [130, 140, 120, 145, 125, 130, 150]

    input_ang = input()
    if input_ang == 'f':
        break
    input_ang = input_ang.split(' ')
    for i, el in enumerate(input_ang):
        if i > 6:
            break
        try:
            el = int(el)
            angles[i] = el
        except:
            pass

    arm_api.send_to_serial(ser, angles)

ser.close()