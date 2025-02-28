import os
from tqdm import tqdm
import argparse
from methods import load_classes, validate_all_labels
from pipeline import process_image

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--src_dir", type=str, default="src_data")
    parser.add_argument("--output_dir", type=str, default="augmented_data")
    parser.add_argument("--classes_config", type=str, default="classes.yaml")
    parser.add_argument("--start_idx", type=int, default=0)
    parser.add_argument("--end_idx", type=int, default=None)
    args = parser.parse_args()

    # Load classes from YAML
    label_to_id, num_classes = load_classes(args.classes_config)
    
    # Validate labels before processing
    validate_all_labels(args.src_dir, label_to_id)
    
    # Create output directories
    os.makedirs(f"{args.output_dir}/images", exist_ok=True)
    os.makedirs(f"{args.output_dir}/labels", exist_ok=True)
    os.makedirs(f"{args.output_dir}/crops", exist_ok=True)
    
    # Process images
    image_files = sorted(os.listdir(f"{args.src_dir}/images"))
    args.end_idx = args.end_idx or len(image_files)
    
    for idx in tqdm(range(args.start_idx, args.end_idx)):
        img_name = image_files[idx]
        img_path = os.path.join(args.src_dir, "images", img_name)
        label_path = os.path.join(args.src_dir, "labels", img_name.replace(".jpg", ".txt"))
        
        if os.path.exists(label_path):
            process_image(
                img_path, 
                label_path, 
                args.output_dir, 
                idx,
                label_to_id  # Pass label mapping
            )