from ultralytics import YOLO

model = YOLO("runs/detect/fruit_detector/weights/best.pt")



model.export(
    format="ncnn",
    imgsz=224,             # Оптимальный размер для баланса точности/производительности
    device='cpu',          # Явное указание CPU-экспорта

)


