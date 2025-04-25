from ultralytics import YOLO
import cv2
import time

def process_camera_stream(model_path, conf_thresh=0.5, imgsz=640, device="cpu"):
    # Загрузка модели
    model = YOLO(model_path)
    
    # Инициализация видеозахвата
    cap = cv2.VideoCapture(0)  # 0 - индекс камеры по умолчанию
    
    if not cap.isOpened():
        print("Ошибка: камера не доступна")
        return

    # Переменные для расчета FPS
    prev_time = 0
    new_time = 0

    while True:
        start_time = time.perf_counter()
        # Захват кадра
        ret, frame = cap.read()
        if not ret:
            print("Ошибка чтения кадра")
            break

        # Конвертация цвета (OpenCV использует BGR, модель ожидает RGB)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Замер времени инференса
        
        
        # Предикт
        results = model.predict(
            source=rgb_frame,
            imgsz=imgsz,
            conf=conf_thresh,
            device=device,
            verbose=False  # Отключаем вывод в консоль для каждого кадра
        )
        
        

        # Визуализация результатов
        annotated_frame = results[0].plot()  # Автоматически конвертирует обратно в BGR
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)

        # Расчет времени обработки
        inference_time = time.perf_counter() - start_time
        fps = 1 / inference_time

        # Отображение FPS
        cv2.putText(
            annotated_frame,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        # Показ кадра
        cv2.imshow("Camera Stream", annotated_frame)

        # Выход по клавише 'q' или ESC
        key = cv2.waitKey(1)
        if key in {ord('q'), 27}:  # 27 - код ESC
            break

    # Освобождение ресурсов
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    process_camera_stream(
        model_path='runs/detect/fruit_detector/weights/best.pt',
        conf_thresh=0.5,
        imgsz=224,  # Рекомендуемый размер для YOLOv8
        device="cpu"  # Меняйте на "cuda" при наличии GPU
    )