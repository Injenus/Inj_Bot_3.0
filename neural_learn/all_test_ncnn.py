import os
import cv2
from ultralytics import YOLO

# Загрузка модели
ncnn_model = YOLO("runs/detect/fruit_detector/weights/best_ncnn_model")

# Классы объектов
class_names = {
    0: "bak",
    1: "tom",
    2: "pear",
    3: "pep",
    4: "lem-ok",
    5: "lem-dead"
}

# Директория с тестовыми изображениями
image_dir = "dataset/images/test"
output_lines = []

# Перебираем изображения
for filename in sorted(os.listdir(image_dir)):
    if not filename.lower().endswith((".jpg", ".jpeg", ".png")):
        continue
    
    image_path = os.path.join(image_dir, filename)
    image = cv2.imread(image_path)
    if image is None:
        continue

    # Инференс
    results = ncnn_model(image, imgsz=224, verbose=False)[0]

    # Статистика по результату
    num_objects = len(results.boxes)
    cls_counts = {}
    lines = []

    for box in results.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        name = class_names.get(cls_id, "unknown")
        lines.append(f"Class ID: {cls_id}, Name: {name}, Confidence: {conf:.2f}")
        cls_counts[name] = cls_counts.get(name, 0) + 1

    # Формирование заголовка
    size = f"{224}x{224}"
    total_label = ", ".join(f"{v} {k}" for k, v in cls_counts.items())
    speed_info = results.speed  # dict: preprocess, inference, postprocess

    header = (
        f"{filename}: {size} {total_label}, {speed_info['inference']:.1f}ms\n"
        f"Speed: {speed_info['preprocess']:.1f}ms preprocess, "
        f"{speed_info['inference']:.1f}ms inference, "
        f"{speed_info['postprocess']:.1f}ms postprocess per image at shape (1, 3, 224, 224)"
    )

    output_lines.append(header)
    output_lines.extend(lines)
    output_lines.append("")  # пустая строка между блоками

# Запись в файл
with open("inference_results.txt", "w") as f:
    f.write("\n".join(output_lines))

print("Результаты записаны в inference_results.txt")
