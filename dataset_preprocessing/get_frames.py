import cv2
import os

video_path = "input_video.mp4"
output_dir = "output_frames"
os.makedirs(output_dir, exist_ok=True)

cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Ошибка при открытии видеофайла")
    exit()

frame_number = 0
saved_count = 0

while True:
    ret, frame = cap.read()
    
    if not ret:
        break
    
    if frame_number % 10 == 0:
        output_path = os.path.join(output_dir, f"{frame_number:05d}.jpg")
        cv2.imwrite(output_path, frame)
        saved_count += 1
    
    frame_number += 1

cap.release()
print(f"Сохранено {saved_count} кадров в директорию: {output_dir}")