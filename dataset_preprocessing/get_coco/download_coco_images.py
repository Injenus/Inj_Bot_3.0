from pycocotools.coco import COCO
import requests
import os
import random

# Настройки
ANNOTATIONS_FILE = r"..\..\test_dataset\annotations\instances_train2017.json"  # Путь к аннотациям
OUTPUT_DIR = r"..\..\test_dataset\raw_images"                                  # Папка для сохранения
CLASSES = ["person", "car", "dog", "cat", "cup"]         # Ваши классы
IMAGES_PER_CLASS = 400                                     # Максимум на класс

# Инициализация COCO API
coco = COCO(ANNOTATIONS_FILE)

# Сбор ID изображений для каждого класса
selected_images = set()
for class_name in CLASSES:
    # Получаем ID класса
    cat_ids = coco.getCatIds(catNms=[class_name])
    if not cat_ids:
        print(f"Класс '{class_name}' не найден в COCO!")
        continue
    
    # Получаем ID изображений с этим классом
    img_ids = coco.getImgIds(catIds=cat_ids)
    random.shuffle(img_ids)
    
    # Выбираем первые N изображений
    selected_images.update(img_ids[:IMAGES_PER_CLASS])

# Скачивание изображений
os.makedirs(OUTPUT_DIR, exist_ok=True)
for img_id in selected_images:
    img_info = coco.loadImgs(img_id)[0]
    img_url = img_info["coco_url"]  # Пример: http://images.cocodataset.org/train2017/000000391895.jpg
    
    # Скачиваем
    try:
        response = requests.get(img_url, timeout=10)
        if response.status_code == 200:
            img_path = os.path.join(OUTPUT_DIR, os.path.basename(img_url))
            with open(img_path, "wb") as f:
                f.write(response.content)
            print(f"Скачано: {img_path}")
        else:
            print(f"Ошибка {response.status_code}: {img_url}")
    except Exception as e:
        print(f"Ошибка при скачивании {img_url}: {str(e)}")