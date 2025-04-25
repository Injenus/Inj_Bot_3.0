import os
import pandas as pd

# Проверка существования папок
print("Существующие папки:", [d for d in os.listdir() if os.path.isdir(d)])

# Проверка правильности названия папки
expected_img_dir = "images_crops"
print(f"\nПапка {expected_img_dir} существует:", os.path.exists(expected_img_dir))



df = pd.read_csv("labels.csv")

# Проверка уникальных расширений и регистра
df['ext'] = df.image_name.str.split('.').str[-1]
print("\nУникальные расширения в CSV:", df.ext.unique())

# Проверка отсутствующих файлов
missing_images = []
for img in df.image_name:
    img_path = os.path.join(expected_img_dir, img)
    if not os.path.exists(img_path):
        missing_images.append(img)

print(f"\nОтсутствует изображений: {len(missing_images)}")
print("Примеры отсутствующих файлов:", missing_images[:5])


# Проверка фактических файлов в папке
actual_files = os.listdir(expected_img_dir)
lowercase_files = [f for f in actual_files if f.lower() != f]
print("\nФайлы с верхним регистром в расширениях:", lowercase_files[:5])



# Проверка соответствия масок
missing_masks = []
for img in df.image_name:
    mask_name = os.path.splitext(img)[0] + ".png"
    mask_path = os.path.join("masks_crops", mask_name)
    if not os.path.exists(mask_path):
        missing_masks.append(mask_name)

print(f"\nОтсутствует масок: {len(missing_masks)}")
print("Примеры отсутствующих масок:", missing_masks[:5])


def find_case_insensitive_path(path):
    dir_path, filename = os.path.split(path)
    actual_dir = dir_path.lower()
    for f in os.listdir(dir_path):
        if f.lower() == filename.lower():
            return os.path.join(dir_path, f)
    return None

# Пример использования:
corrected_path = find_case_insensitive_path("images_crops/lem-ok-8728.JPG")
print("\nКорректный путь:", corrected_path)