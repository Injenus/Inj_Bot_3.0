import cv2

import methods

def process_image(image_path, label_path, output_dir, img_idx):
    # [1] Загрузка и Letterbox
    orig_image = cv2.imread(image_path)
    image, scale, (dw, dh) = letterbox(orig_image)
    orig_boxes = read_yolo_annotation(label_path, orig_image.shape[1], orig_image.shape[0])
    
    # Конвертация боксов в Letterbox-координаты
    boxes = []
    for box in orig_boxes:
        class_id, x1, y1, x2, y2 = box
        x1 = x1 * scale + dw / 2
        y1 = y1 * scale + dh / 2
        x2 = x2 * scale + dw / 2
        y2 = y2 * scale + dh / 2
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
            save_yolo_annotation(valid_boxes, 320, 320, f"{output_dir}/labels/{img_name.replace('.jpg', '.txt')}")