# test_pt.py
from ultralytics import YOLO
import cv2

img_path = 'dataset/images/test/'


def test_pt_model(model_path, image_path, conf_thresh=0.5):
    # Загрузка модели
    model = YOLO(model_path)
    
    # Предсказание
    results = model.predict(image_path, imgsz=224, conf=conf_thresh, device='cpu')
    
    # Визуализация
    annotated_img = results[0].plot(line_width=1, font_size=8)
    
    # Сохранение и показ
    cv2.imwrite('pt_result.jpg', annotated_img)
    cv2.imshow('PyTorch Detection', annotated_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_pt_model(
        model_path='runs/detect/fruit_detector/weights/best.pt',
        image_path=img_path +'pep-9087.jpg',
        conf_thresh=0.1
    )