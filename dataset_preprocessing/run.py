import os

if __name__ == "__main__":
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