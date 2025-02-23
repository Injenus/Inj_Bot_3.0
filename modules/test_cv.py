import cv2

def capture_frame(device_index, filename):
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        print(f"Не удалось открыть камеру с индексом {device_index}")
        return
    ret, frame = cap.read()
    if ret:
        i = 0
        while i < 60:
            ret, frame = cap.read()
            i += 1
        cv2.imwrite(filename, frame)
        print(f"Снимок сохранен в {filename}")
    else:
        print(f"Не удалось получить кадр с камеры {device_index}")
    cap.release()

for i in range(36):
    capture_frame(i, f'{i}_.jpg')