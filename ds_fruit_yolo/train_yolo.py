import os
import cv2
import pandas as pd
import shutil
from pathlib import Path
from sklearn.model_selection import train_test_split
from ultralytics import YOLO

def prepare_yolo_dataset():
    # Создаем директории YOLO
    BASE_DIR = os.path.abspath("dataset")
    os.makedirs(os.path.join(BASE_DIR, "images/train"), exist_ok=True)
    os.makedirs(os.path.join(BASE_DIR, "images/val"), exist_ok=True)
    os.makedirs(os.path.join(BASE_DIR, "images/test"), exist_ok=True)
    os.makedirs(os.path.join(BASE_DIR, "labels/train"), exist_ok=True)
    os.makedirs(os.path.join(BASE_DIR, "labels/val"), exist_ok=True)
    os.makedirs(os.path.join(BASE_DIR, "labels/test"), exist_ok=True)

    # Читаем CSV
    df = pd.read_csv("labels.csv")
    
    # Убедимся, что все файлы существуют
    df = df[df["image_name"].apply(lambda x: os.path.exists(f"images_crops/{x}"))]

    # Разделяем данные
    train_df, temp_df = train_test_split(df, test_size=0.2, random_state=42, stratify=df["class"])
    val_df, test_df = train_test_split(temp_df, test_size=0.5, random_state=42, stratify=temp_df["class"])

    # Функция для обработки данных
    def process_image(row, subset):
        img_name = row["image_name"]
        base_name = os.path.splitext(img_name)[0]
        class_id = row["class"]
        
        # Копируем изображение
        img_src = f"images_crops/{img_name}"
        img_dst = f"{BASE_DIR}/images/{subset}/{img_name}"
        shutil.copy(img_src, img_dst)
        
        # Обрабатываем маску
        mask_path = f"masks_crops/{base_name}.png"
        if not os.path.exists(mask_path):
            raise FileNotFoundError(f"Mask {mask_path} not found")
        
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            print(f"Warning: No contours found in {mask_path}")
            return

        # Создаем аннотации
        label_path = f"{BASE_DIR}/labels/{subset}/{base_name}.txt"
        with open(label_path, "w") as f:
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                img = cv2.imread(img_src)
                height, width = img.shape[:2]
                x_center = (x + w/2) / width
                y_center = (y + h/2) / height
                w_norm = w / width
                h_norm = h / height
                f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}\n")

    # Обрабатываем данные
    for df, subset in zip([train_df, val_df, test_df], ["train", "val", "test"]):
        for _, row in df.iterrows():
            process_image(row, subset)

    # Создаем YAML-конфиг с абсолютными путями
    yaml_content = f"""path: {BASE_DIR}
train: images/train
val: images/val
test: images/test

names:
  0: bak
  1: tom
  2: pear
  3: pep
  4: lem-ok
  5: lem-dead
"""

    with open("dataset.yaml", "w") as f:
        f.write(yaml_content)

def train_yolo():
    model = YOLO("yolov8n.pt")
    
    # Запускаем обучение
    results = model.train(
        data=os.path.abspath("dataset.yaml"),
        epochs=1,
        imgsz=320,
        batch=16,
        device="cpu",
        exist_ok=False,
        name="fruit_detector",
        save=True,
        verbose=True
    )
    
    # Определяем путь к лучшей модели
    best_pt_path = Path(results.save_dir) / "weights" / "best.pt"
    
    if not best_pt_path.exists():
        raise FileNotFoundError(f"Model weights not found at {best_pt_path}")
    
    # Экспорт в TFLite
    best_model = YOLO(best_pt_path)
    
    # Производим экспорт с явным указанием имени
    export_results = best_model.export(
        format="tflite",
        imgsz=320,
        int8=False,
        simplify=True,
        opset=13,
        name="raspberry_export"
    )
    
    # Автоматически определяем путь к экспортированному файлу
    if not export_results:
        raise RuntimeError("TFLite export failed: no output files")
    
    tflite_path = Path(export_results[0])
    
    # Переносим файл в корень проекта
    final_path = Path("model.tflite").resolve()
    shutil.copy(tflite_path, final_path)
    
    # Проверка результата
    if final_path.exists():
        print(f"\n✅ TFLite model successfully saved to: {final_path}")
        print(f"File size: {final_path.stat().st_size / 1e6:.1f} MB")
    else:
        print("\n❌ TFLite export failed!")

if __name__ == "__main__":
    prepare_yolo_dataset()
    train_yolo()