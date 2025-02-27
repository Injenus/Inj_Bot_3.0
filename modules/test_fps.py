import cv2
import time

cap = cv2.VideoCapture(0)  # Используйте ваш индекс
cap.set(cv2.CAP_PROP_FPS, 30)  # Явно запросите 30 FPS
prev_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    print(f"FPS: {1 / (time.time() - prev_time):.1f}")
    prev_time = time.time()