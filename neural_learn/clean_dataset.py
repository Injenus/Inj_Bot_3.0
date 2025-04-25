import os
import pandas as pd

def clean_dataset():
    # Пути к данным
    csv_path = "labels.csv"
    images_dir = "images_crops"
    masks_dir = "masks_crops"
    output_csv = "labels_cleaned.csv"

    # Загрузка CSV
    df = pd.read_csv(csv_path)
    
    # Приведение имен к нижнему регистру
    df['image_name'] = df['image_name'].str.lower()
    
    # Поиск существующих файлов
    valid_records = []
    
    for _, row in df.iterrows():
        img_name = row['image_name']
        base_name = os.path.splitext(img_name)[0]
        
        # Проверка изображения
        img_path = os.path.join(images_dir, img_name)
        if not os.path.exists(img_path):
            # Попробовать найти с другим расширением
            for ext in ['.jpg', '.jpeg', '.png', '.JPG', '.JPEG']:
                alt_path = os.path.join(images_dir, f"{base_name}{ext}")
                if os.path.exists(alt_path):
                    img_path = alt_path
                    break
            else:
                continue  # Пропустить если не найдено

        # Проверка маски
        mask_path = os.path.join(masks_dir, f"{base_name}.png")
        if not os.path.exists(mask_path):
            # Попробовать найти с другим расширением
            for ext in ['.png', '.PNG']:
                alt_mask_path = os.path.join(masks_dir, f"{base_name}{ext}")
                if os.path.exists(alt_mask_path):
                    mask_path = alt_mask_path
                    break
            else:
                continue  # Пропустить если маска не найдена

        valid_records.append(row)

    # Создание чистого датасета
    clean_df = pd.DataFrame(valid_records)
    
    # Сохранение нового CSV
    clean_df.to_csv(output_csv, index=False)
    
    # Переименование файлов для единообразия
    for img_name in clean_df['image_name']:
        base = os.path.splitext(img_name)[0]
        src_img = os.path.join(images_dir, img_name)
        
        # Стандартизация расширения изображений
        if not os.path.exists(src_img):
            for ext in ['.jpg', '.jpeg', '.JPG', '.JPEG']:
                candidate = os.path.join(images_dir, f"{base}{ext}")
                if os.path.exists(candidate):
                    os.rename(candidate, os.path.join(images_dir, f"{base}.jpg"))
        
        # Стандартизация масок
        mask_src = os.path.join(masks_dir, f"{base}.png")
        if not os.path.exists(mask_src):
            for ext in ['.PNG']:
                candidate = os.path.join(masks_dir, f"{base}{ext}")
                if os.path.exists(candidate):
                    os.rename(candidate, mask_src)

    print(f"Очистка завершена. Валидных записей: {len(clean_df)}")
    print(f"Новый файл: {output_csv}")

if __name__ == "__main__":
    clean_dataset()