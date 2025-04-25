from ultralytics import YOLO
import cv2

# Загрузка модели NCNN
ncnn_model = YOLO("/home/inj/Inj_Bot_3.0/neurals/weights_fisrt_harv/best_ncnn_model")

# Классы объектов (как в dataset.yaml)
class_names = {
    0: "bak",
    1: "tom",
    2: "pear",
    3: "pep",
    4: "lem-ok",
    5: "lem-dead"
}

# Путь к изображению
image_path = '/home/inj/Inj_Bot_3.0/y_harv_test/dataset/images/test/lem-dead-9285.jpg'
image = cv2.imread(image_path)

# Запуск инференса
results = ncnn_model(image, imgsz=224)[0]

# Обработка результатов
for box in results.boxes:
    cls_id = int(box.cls[0])
    conf = float(box.conf[0])
    x1, y1, x2, y2 = map(int, box.xyxy[0])

    # Вывод в консоль
    print(f"Class ID: {cls_id}, Name: {class_names.get(cls_id, 'unknown')}, Confidence: {conf:.2f}")

    # Подпись и рамка
    label = f"{class_names.get(cls_id, 'unknown')} {conf:.2f}"
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(image, label, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# # Отображение результата
# cv2.imshow("Detected Objects", image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
