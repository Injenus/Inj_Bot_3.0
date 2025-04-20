import cv2
import os

# Настройки
input_folder = "images"      # Папка с исходными изображениями
output_folder = "images_crop"   # Папка для сохранения результатов

# Создаем папку для результатов, если её нет
os.makedirs(output_folder, exist_ok=True)

# Обрабатываем все PNG файлы в папке
for filename in os.listdir(input_folder):
    if filename.lower().endswith(".png"):
        # Читаем изображение
        img_path = os.path.join(input_folder, filename)
        image = cv2.imread(img_path)
        
        if image is not None:
            base_name = os.path.splitext(filename)[0]

            resized = cv2.resize(image, (int(image.shape[1]/3.3), int(image.shape[0]/3.3)))
            
            output_name = f"{base_name}_res.png"
            output_path = os.path.join(output_folder, output_name)
            
            # Сохраняем результат
            cv2.imwrite(output_path, resized)
            
            print(f"Обработано: {filename}")
        else:
            print(f"Ошибка чтения: {filename}")

print("Все изображения обработаны!")