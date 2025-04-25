import os
import cv2
import pandas as pd
import shutil
from sklearn.model_selection import train_test_split
from ultralytics import YOLO
import numpy as np

def prepare_yolo_dataset():
    # Определяем пути относительно расположения скрипта
    script_dir = os.path.dirname(os.path.abspath(__file__))
    BASE_DIR = os.path.join(script_dir, "dataset")
    IMAGE_SRC_DIR = os.path.join(script_dir, "images_crops")
    MASK_SRC_DIR = os.path.join(script_dir, "masks_crops")
    
    # Создаем директории YOLO
    for split in ["train", "val", "test"]:
        os.makedirs(os.path.join(BASE_DIR, "images", split), exist_ok=True)
        os.makedirs(os.path.join(BASE_DIR, "labels", split), exist_ok=True)

    # Читаем CSV
    df = pd.read_csv(os.path.join(script_dir, "labels.csv"))
    
    # Проверка существования файлов
    df = df[df["image_name"].apply(lambda x: os.path.exists(os.path.join(IMAGE_SRC_DIR, x)))]
    
    # Уникальные изображения для корректного разделения
    unique_images = df["image_name"].unique()
    
    # Стратификация по первому классу в изображении (упрощенно)
    image_classes = df.groupby("image_name")["class"].first().values
    
    # Разделение данных на уровне изображений
    train_images, temp_images = train_test_split(
        unique_images, 
        test_size=0.2, 
        random_state=42, 
        stratify=image_classes
    )
    
    val_images, test_images = train_test_split(
        temp_images,
        test_size=0.5,
        random_state=42,
        stratify=image_classes[np.isin(unique_images, temp_images)]
    )

    # Создаем DataFrame для каждого сета
    train_df = df[df["image_name"].isin(train_images)]
    val_df = df[df["image_name"].isin(val_images)]
    test_df = df[df["image_name"].isin(test_images)]

    def process_image(image_name, subset):
        base_name = os.path.splitext(image_name)[0]
        img_src = os.path.join(IMAGE_SRC_DIR, image_name)
        img_dst = os.path.join(BASE_DIR, "images", subset, image_name)
        
        # Копируем изображение
        shutil.copy(img_src, img_dst)
        
        # Обрабатываем маску
        mask_path = os.path.join(MASK_SRC_DIR, f"{base_name}.png")
        if not os.path.exists(mask_path):
            raise FileNotFoundError(f"Mask {mask_path} not found")
        
        # Читаем изображение для размеров
        img = cv2.imread(img_src)
        if img is None:
            raise ValueError(f"Error reading image {img_src}")
        height, width = img.shape[:2]
        
        # Создаем файл аннотаций
        label_path = os.path.join(BASE_DIR, "labels", subset, f"{base_name}.txt")
        with open(label_path, "w") as f:
            # Обрабатываем все объекты для этого изображения
            for _, row in df[df["image_name"] == image_name].iterrows():
                class_id = row["class"]
                mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
                
                # Пороговое значение для бинаризации маски
                _, binary_mask = cv2.threshold(mask, 128, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for cnt in contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                    x_center = (x + w/2) / width
                    y_center = (y + h/2) / height
                    w_norm = w / width
                    h_norm = h / height
                    f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}\n")

    # Обработка данных
    for subset, images in zip(["train", "val", "test"], [train_images, val_images, test_images]):
        for image_name in images:
            process_image(image_name, subset)

    # Создаем YAML-конфиг
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
    with open(os.path.join(script_dir, "dataset.yaml"), "w") as f:
        f.write(yaml_content)

def train_yolo():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model = YOLO("yolov8n.pt")
    
    model.train(
        data=os.path.join(script_dir, "dataset.yaml"),
        epochs=100,
        imgsz=224,
        batch=16,
        device="cpu",
        name="fruit_detector",
        lr0=0.001,
        pretrained=True,
    )

if __name__ == "__main__":
    prepare_yolo_dataset()
    train_yolo()