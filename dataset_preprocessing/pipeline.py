import cv2

from methods import *

def process_image(image_path, label_path, output_dir, img_idx, label_to_id):
    orig_image = cv2.imread(image_path)
    if orig_image is None:
        print(f"Error loading image: {image_path}")
        return
    
    orig_h, orig_w = orig_image.shape[:2]
    
    if not os.path.exists(label_path):
        print(f"Missing annotation: {label_path}")
        return
    
    if label_path.endswith('.json'):
        orig_boxes = read_labelstudio_annotation(label_path, orig_w, orig_h, label_to_id)
    else:
        orig_boxes = read_yolo_annotation(label_path, orig_w, orig_h)
    
    # Letterbox преобразование
    image, scale, (dw, dh) = letterbox(orig_image)
    new_h, new_w = 320, 320  # Размер после Letterbox
    
    # Конвертация боксов
    boxes = []
    for box in orig_boxes:
        class_id, x1, y1, x2, y2 = box
        # Масштабирование и смещение
        x1 = x1 * scale + dw / 2
        y1 = y1 * scale + dh / 2
        x2 = x2 * scale + dw / 2
        y2 = y2 * scale + dh / 2
        
        if x2 <= x1 or y2 <= y1:  # Фильтрация невалидных боксов
            continue
        
        boxes.append([class_id, x1, y1, x2, y2])
    
    # [2] Аугментации
    angles = get_random_rotation_angles(n=2)
    directions = get_random_perspective_directions(n=2)
    scale_aug = get_random_scale()
    brightness = get_random_brightness()
    hsv_factors = get_random_hsv_factors()
    
    for angle in angles:
        rotated, rotated_boxes = rotate_image(image, angle, boxes)
        flipped, flipped_boxes = horizontal_flip(rotated, rotated_boxes, 320)
        
        for direction in directions:
            warped, warped_boxes = perspective_shift(flipped, flipped_boxes, direction)
            scaled, scaled_boxes = scale_image(warped, scale_aug, warped_boxes)
            bright_img = adjust_brightness_lab(scaled, brightness)
            hsv_img = hsv_augmentation(bright_img, hsv_factors['h'], hsv_factors['s'], hsv_factors['v'])
            noisy_img = add_horizontal_noise(hsv_img)
            
            # [3] Фильтрация боксов перед кропами
            valid_boxes = [b for b in scaled_boxes if is_valid_box(b, 320, 320)]
            
            # [4] Генерация кропов
            generate_crops(
                image=noisy_img,
                boxes=valid_boxes,
                output_dir=f"{output_dir}/crops",
                base_name=f"aug_{img_idx}_rot{angle}_dir{direction}_scale{scale_aug}"
            )
            
            # [5] Сохранение финальных данных
            img_name = f"aug_{img_idx}_rot{angle}_dir{direction}_scale{scale_aug}_bright{brightness}_h{hsv_factors['h']}s{hsv_factors['s']}v{hsv_factors['v']}.jpg"
            
            cv2.imwrite(f"{output_dir}/images/{img_name}", noisy_img)
            
            # YOLO-аннотации
            save_yolo_annotation(
                boxes=valid_boxes,          # Обработанные боксы
                img_w=320,                  # Ширина после letterbox
                img_h=320,                  # Высота после letterbox
                save_path=f"{output_dir}/labels/{img_name.replace('.jpg', '.txt')}"
            )