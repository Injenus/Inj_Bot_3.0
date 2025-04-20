import torch
import torch.nn as nn
from torchvision import transforms
import cv2
import os


DATA_DIR = 'dataset'  # Корневая директория с папками images и masks
IMAGES_DIR = 'dataset/images'
MASKS_DIR = 'dataset/masks'
LABELS_FILE = 'dataset/labels.csv'
BATCH_SIZE = 8
LEARNING_RATE = 0.001
EPOCHS = 10
NUM_CLASSES = 6  # Количество ваших классов объектов
RANDOM_SEED = 42
torch.manual_seed(RANDOM_SEED)


class SimpleCNN(nn.Module):
    def __init__(self, num_classes):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=1)
        self.relu1 = nn.ReLU()
        self.maxpool1 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.relu2 = nn.ReLU()
        self.maxpool2 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.fc = nn.Linear(64 * 56 * 56, num_classes) # Размерность после сверток и пулингов

    def forward(self, x):
        x = self.maxpool1(self.relu1(self.conv1(x)))
        x = self.maxpool2(self.relu2(self.conv2(x)))
        x = x.view(x.size(0), -1) # Flatten
        x = self.fc(x)
        return x
    
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224, 224)),  # Стандартный размер для многих сетей
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Стандартные значения для ImageNet
])
    
loaded_model = SimpleCNN(NUM_CLASSES)
loaded_model.load_state_dict(torch.load('dump_classifier.pth'))
loaded_model.eval()

def predict_by_photo(photo):
  # sample_image_path = os.path.join(IMAGES_DIR, 'example.jpg') # Замените на путь к вашему изображению
  # sample_image = cv2.imread(sample_image_path)
  sample_image = cv2.cvtColor(photo, cv2.COLOR_BGR2RGB)
  sample_image_tensor = transform(sample_image).unsqueeze(0) # Добавляем размерность батча
  with torch.no_grad():
    output = loaded_model(sample_image_tensor)
    _, predicted_class = torch.max(output, 1)
    predicted_label = predicted_class.item()

  return predicted_label

