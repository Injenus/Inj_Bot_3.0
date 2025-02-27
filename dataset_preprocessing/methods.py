import os
import cv2
import numpy as np
import random
from tqdm import tqdm

def read_yolo_annotation(label_path, orig_w, orig_h):
    boxes = []
    with open(label_path, 'r') as f:
        for line in f:
            class_id, xc, yc, bw, bh = map(float, line.strip().split())
            # Конвертация YOLO в абсолютные координаты
            x1 = (xc - bw/2) * orig_w
            y1 = (yc - bh/2) * orig_h
            x2 = (xc + bw/2) * orig_w
            y2 = (yc + bh/2) * orig_h
            boxes.append([int(class_id), x1, y1, x2, y2])
    return boxes

def save_yolo_annotation(boxes, img_w, img_h, save_path):
    with open(save_path, 'w') as f:
        for box in boxes:
            class_id, x1, y1, x2, y2 = box
            # Конвертация в YOLO-формат
            xc = ((x1 + x2) / 2) / img_w
            yc = ((y1 + y2) / 2) / img_h
            bw = (x2 - x1) / img_w
            bh = (y2 - y1) / img_h
            f.write(f"{class_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")

def is_valid_box(box, img_w, img_h, min_size=5):
    _, x1, y1, x2, y2 = box
    return (x2 > x1 + min_size) and (y2 > y1 + min_size) and \
           (x1 >= 0) and (x2 <= img_w) and (y1 >= 0) and (y2 <= img_h)
            

def horizontal_flip(image, boxes, img_width):
    flipped = cv2.flip(image, 1)
    new_boxes = []
    for box in boxes:
        class_id, x1, y1, x2, y2 = box
        new_x1 = img_width - x2
        new_x2 = img_width - x1
        new_boxes.append([class_id, new_x1, y1, new_x2, y2])
    return flipped, new_boxes


def rotate_image(image, angle, boxes):
    h, w = image.shape[:2]
    center = (w/2, h/2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))
    
    new_boxes = []
    for box in boxes:
        class_id, x1, y1, x2, y2 = box
        # Поворот всех 4 точек бокса
        points = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        ones = np.ones(shape=(4, 1))
        points_ones = np.hstack([points, ones])
        transformed_points = M.dot(points_ones.T).T
        # Новые координаты
        new_x1 = max(0, np.min(transformed_points[:, 0]))
        new_y1 = max(0, np.min(transformed_points[:, 1]))
        new_x2 = min(w, np.max(transformed_points[:, 0]))
        new_y2 = min(h, np.max(transformed_points[:, 1]))
        new_boxes.append([class_id, new_x1, new_y1, new_x2, new_y2])
    
    return rotated, new_boxes


def generate_crops(image, boxes, output_dir, base_name):
    os.makedirs(output_dir, exist_ok=True)
    for idx, box in enumerate(boxes):
        class_id, x1, y1, x2, y2 = map(int, box)
        crop = image[y1:y2, x1:x2]
        if crop.size == 0:
            continue
        cv2.imwrite(f"{output_dir}/{base_name}_crop_{idx}.jpg", crop)


def scale_image(image, scale_factor, boxes):
    h, w = image.shape[:2]
    new_w = int(w * scale_factor)
    new_h = int(h * scale_factor)
    scaled = cv2.resize(image, (new_w, new_h))
    
    new_boxes = []
    for box in boxes:
        class_id, x1, y1, x2, y2 = box
        new_x1 = x1 * scale_factor
        new_y1 = y1 * scale_factor
        new_x2 = x2 * scale_factor
        new_y2 = y2 * scale_factor
        new_boxes.append([class_id, new_x1, new_y1, new_x2, new_y2])
    
    return scaled, new_boxes


def adjust_brightness_lab(image, factor):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = np.clip(l * factor, 0, 255).astype(np.uint8)
    merged = cv2.merge([l, a, b])
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)


def add_horizontal_noise(image, intensity=0.1):
    h, w = image.shape[:2]
    noise = np.zeros((h, w, 3), dtype=np.uint8)
    
    # Случайные цветные полосы
    for y in range(h):
        if random.random() < intensity:
            color = [random.randint(0, 255) for _ in range(3)]
            noise[y, :] = color
    
    return cv2.addWeighted(image, 0.8, noise, 0.2, 0)


def hsv_augmentation(image, h_factor, s_factor, v_factor):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    # Применение коэффициентов
    h = np.clip(h * h_factor, 0, 179).astype(np.uint8)  # Hue в диапазоне [0, 179]
    s = np.clip(s * s_factor, 0, 255).astype(np.uint8)
    v = np.clip(v * v_factor, 0, 255).astype(np.uint8)
    
    merged = cv2.merge([h, s, v])
    return cv2.cvtColor(merged, cv2.COLOR_HSV2BGR)


def perspective_shift(image, boxes, direction, intensity=0.2):
    h, w = image.shape[:2]
    pts_src = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    
    # Смещение точек для имитации наклона
    if direction == 'up':
        pts_dst = np.array([[intensity*w, 0], [w-intensity*w, 0], [w, h], [0, h]], dtype=np.float32)
    elif direction == 'down':
        pts_dst = np.array([[0, 0], [w, 0], [w-intensity*w, h], [intensity*w, h]], dtype=np.float32)
    elif direction == 'left':
        pts_dst = np.array([[0, intensity*h], [w, 0], [w, h], [0, h-intensity*h]], dtype=np.float32)
    elif direction == 'right':
        pts_dst = np.array([[0, 0], [w, intensity*h], [w, h-intensity*h], [0, h]], dtype=np.float32)

    # Вычисление матрицы гомографии
    M = cv2.getPerspectiveTransform(pts_src, pts_dst)
    warped = cv2.warpPerspective(image, M, (w, h), borderMode=cv2.BORDER_REFLECT)

    # Преобразование боксов
    new_boxes = []
    for box in boxes:
        class_id, x1, y1, x2, y2 = box
        points = np.array([
            [x1, y1], [x2, y1], [x2, y2], [x1, y2]
        ], dtype=np.float32)
        
        # Применение перспективного преобразования
        transformed = cv2.perspectiveTransform(points.reshape(1, -1, 2), M).squeeze()
        
        # Новый ограничивающий прямоугольник
        new_x1 = max(0, np.min(transformed[:, 0]))
        new_y1 = max(0, np.min(transformed[:, 1]))
        new_x2 = min(w, np.max(transformed[:, 0]))
        new_y2 = min(h, np.max(transformed[:, 1]))
        
        if new_x2 - new_x1 > 5 and new_y2 - new_y1 > 5:  # Фильтр мелких артефактов
            new_boxes.append([class_id, new_x1, new_y1, new_x2, new_y2])

    return warped, new_boxes


def letterbox(image, target_size=320, pad_value=114):
    h, w = image.shape[:2]
    scale = min(target_size / h, target_size / w)
    new_h, new_w = int(h * scale), int(w * scale)
    
    # Resize с сохранением пропорций
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    # Добавление паддинга
    dw = target_size - new_w
    dh = target_size - new_h
    top, bottom = dh // 2, dh - (dh // 2)
    left, right = dw // 2, dw - (dw // 2)
    
    padded = cv2.copyMakeBorder(
        resized, 
        top, bottom, left, right,
        cv2.BORDER_CONSTANT,
        value=(pad_value, pad_value, pad_value))
    
    return padded, scale, (dw, dh)

#-------------------------------------------------------------------------------------------

def get_random_rotation_angles(angles_range=(-180, 180), step=30, n=2):
    all_angles = list(range(angles_range[0], angles_range[1]+1, step))
    return random.sample(all_angles, n)

def get_random_perspective_directions(n=2):
    return random.sample(['up', 'down', 'left', 'right'], n)

def get_random_scale(scales=[0.75, 1.25]):
    return random.choice(scales)

def get_random_brightness():
    return random.choice([0.5, 0.75, 1.25, 1.5])

def get_random_hsv_factors():
    return {
        'h': random.choice([0.85, 1.25]),
        's': random.choice([0.85, 1.25]), 
        'v': random.choice([0.85, 1.25])
    }