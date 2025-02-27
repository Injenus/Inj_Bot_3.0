import os
from tqdm import tqdm
import argparse

from pipeline import *

if __name__ == "__main__":
    # 1. Инициализация парсера
    parser = argparse.ArgumentParser()
    parser.add_argument("--src_dir", type=str, default="src_data")
    parser.add_argument("--output_dir", type=str, default="augmented_data")
    parser.add_argument("--start_idx", type=int, default=0)
    parser.add_argument("--end_idx", type=int, default=None)
    args = parser.parse_args()

    # Создание выходных папок
    os.makedirs(f"{args.output_dir}/images", exist_ok=True)
    os.makedirs(f"{args.output_dir}/labels", exist_ok=True)
    os.makedirs(f"{args.output_dir}/crops", exist_ok=True)
    
    # Получение списка изображений
    image_files = sorted(os.listdir(f"{args.src_dir}/images"))
    if args.end_idx is None:
        args.end_idx = len(image_files)
    
    # Обработка каждого изображения
    for idx in tqdm(range(args.start_idx, args.end_idx)):
        img_name = image_files[idx]
        img_path = os.path.join(args.src_dir, "images", img_name)
        label_path = os.path.join(args.src_dir, "labels", img_name.replace(".jpg", ".txt"))
        
        if not os.path.exists(label_path):
            continue
            
        process_image(img_path, label_path, args.output_dir, idx)