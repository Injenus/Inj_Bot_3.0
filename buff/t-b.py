import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
import timm
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import albumentations as A
import os
from tqdm import tqdm

class DualTaskModel(nn.Module):
  def __init__(self, num_classes=3):
    super().__init__()
    
    # Энкодер с явным указанием выходного разрешения
    self.encoder = timm.create_model(
      'efficientnet_b0',
      pretrained=True,
      features_only=True,
      out_indices=(4,)  # Фичи с последнего слоя (8x8 для входа 256x256)
    )
    
    # Декодер с точной апсемплирующей структурой
    self.decoder = nn.Sequential(
      # Блок 1: 8x8 -> 16x16
      nn.ConvTranspose2d(320, 256, 3, stride=2, padding=1, output_padding=1),
      nn.BatchNorm2d(256),
      nn.ReLU(),
      
      # Блок 2: 16x16 -> 32x32
      nn.ConvTranspose2d(256, 128, 3, stride=2, padding=1, output_padding=1),
      nn.BatchNorm2d(128),
      nn.ReLU(),
      
      # Блок 3: 32x32 -> 64x64
      nn.ConvTranspose2d(128, 64, 3, stride=2, padding=1, output_padding=1),
      nn.BatchNorm2d(64),
      nn.ReLU(),
      
      # Блок 4: 64x64 -> 128x128
      nn.ConvTranspose2d(64, 32, 3, stride=2, padding=1, output_padding=1),
      nn.BatchNorm2d(32),
      nn.ReLU(),
      
      # Блок 5: 128x128 -> 256x256
      nn.ConvTranspose2d(32, 16, 3, stride=2, padding=1, output_padding=1),
      nn.BatchNorm2d(16),
      nn.ReLU(),
      
      # Финал
      nn.Conv2d(16, 1, kernel_size=1)
    )

    # Классификационная головка
    self.cls_head = nn.Sequential(
        nn.AdaptiveAvgPool2d(1),
        nn.Flatten(),
        nn.Linear(320, num_classes)
    )

  def forward(self, x):
      # Энкодер: [B,3,256,256] -> [B,320,8,8]
      features = self.encoder(x)[0]
      
      # Декодер: [B,320,8,8] -> [B,1,256,256]
      seg_output = self.decoder(features)
      
      # Классификация
      cls_output = self.cls_head(features)
      
      return seg_output, cls_output
    
class ObjectDataset(Dataset):
  def __init__(self, root_dir, split='train', img_size=256):
    self.root_dir = root_dir
    self.img_size = img_size
    self.labels_df = pd.read_csv(os.path.join(root_dir, 'labels.csv'))
    
    # Разделение данных
    split_idx = int(0.8 * len(self.labels_df))
    if split == 'train':
      self.data = self.labels_df.iloc[:split_idx]
    else:
      self.data = self.labels_df.iloc[split_idx:]
    
    # Аугментации
    self.transform = A.Compose([
      A.Resize(img_size, img_size),
      A.HorizontalFlip(p=0.5),
      A.RandomBrightnessContrast(p=0.3),
      A.GaussianBlur(p=0.1),
      A.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

  def __len__(self):
      return len(self.data)

  def __getitem__(self, idx):
    row = self.data.iloc[idx]
    
    # Загрузка изображения
    img_path = os.path.join(self.root_dir, 'images', row['image_name'])
    image = cv2.imread(img_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # Загрузка маски
    mask_path = os.path.join(self.root_dir, 'masks', row['image_name'].replace('.jpg', '.png'))
    mask = cv2.imread(mask_path, 0)
    
    # Применение аугментаций
    transformed = self.transform(image=image, mask=mask)
    image = transformed['image']
    mask = transformed['mask']
    
    # Преобразование в тензоры
    image = torch.tensor(image).permute(2, 0, 1).float()
    mask = torch.tensor(mask).unsqueeze(0).float()
    label = torch.tensor(row['class']).long()
    
    return image, mask, label
  
def train_model():
  # Параметры
  BATCH_SIZE = 8
  EPOCHS = 20
  LR = 1e-4
  
  # Датасеты
  train_dataset = ObjectDataset('dataset', 'train')
  val_dataset = ObjectDataset('dataset', 'val')
  
  train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
  val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE)
  
  # Модель и оптимизатор
  device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
  model = DualTaskModel(num_classes=3).to(device)
  optimizer = optim.Adam(model.parameters(), lr=LR)
  
  # Функции потерь
  seg_criterion = nn.BCEWithLogitsLoss()
  cls_criterion = nn.CrossEntropyLoss()
  
  # История обучения
  history = {'train_loss': [], 'val_loss': [], 'cls_acc': []}
  
  for epoch in range(EPOCHS):
    model.train()
    train_loss = 0
    
    # Обучение
    for images, masks, labels in tqdm(train_loader, desc=f'Epoch {epoch+1}/{EPOCHS}'):
      images = images.to(device)
      masks = masks.to(device)
      labels = labels.to(device)
      
      optimizer.zero_grad()
      
      seg_out, cls_out = model(images)
      loss_seg = seg_criterion(seg_out, masks)
      loss_cls = cls_criterion(cls_out, labels)
      loss = 0.7*loss_seg + 0.3*loss_cls
      
      loss.backward()
      optimizer.step()
      
      train_loss += loss.item()
    
    # Валидация
    model.eval()
    val_loss = 0
    correct = 0
    total = 0
    
    with torch.no_grad():
      for images, masks, labels in val_loader:
        images = images.to(device)
        masks = masks.to(device)
        labels = labels.to(device)
        
        seg_out, cls_out = model(images)
        loss_seg = seg_criterion(seg_out, masks)
        loss_cls = cls_criterion(cls_out, labels)
        loss = 0.7*loss_seg + 0.3*loss_cls
        
        val_loss += loss.item()
        _, predicted = torch.max(cls_out.data, 1)
        correct += (predicted == labels).sum().item()
        total += labels.size(0)
    
    # Сохранение метрик
    history['train_loss'].append(train_loss/len(train_loader))
    history['val_loss'].append(val_loss/len(val_loader))
    history['cls_acc'].append(100*correct/total)
    
    print(f"Epoch {epoch+1}: "
      f"Train Loss: {history['train_loss'][-1]:.4f}, "
      f"Val Loss: {history['val_loss'][-1]:.4f}, "
      f"Accuracy: {history['cls_acc'][-1]:.2f}%")
  
  # Графики обучения
  plt.figure(figsize=(12, 4))
  plt.subplot(121)
  plt.plot(history['train_loss'], label='Train Loss')
  plt.plot(history['val_loss'], label='Val Loss')
  plt.legend()
  
  plt.subplot(122)
  plt.plot(history['cls_acc'], label='Accuracy')
  plt.legend()
  plt.show()
  
  return model

def visualize_predictions(model, device, num_samples=3):
  dataset = ObjectDataset('dataset')
  model.eval()
  
  plt.figure(figsize=(15, 5))
  for i in range(num_samples):
    image, true_mask, true_label = dataset[i]
    
    with torch.no_grad():
        seg_pred, cls_pred = model(image.unsqueeze(0).to(device))
    
    # Постобработка
    pred_mask = (torch.sigmoid(seg_pred) > 0.5).cpu().squeeze()
    pred_label = torch.argmax(cls_pred).item()
    
    # Визуализация
    plt.subplot(3, 3, i*3+1)
    plt.imshow(image.permute(1, 2, 0).numpy() * [0.229, 0.224, 0.225] + [0.485, 0.456, 0.406])
    plt.title(f"Original Image\nTrue Class: {true_label}")
    plt.axis('off')
    
    plt.subplot(3, 3, i*3+2)
    plt.imshow(true_mask.squeeze(), cmap='gray')
    plt.title("True Mask")
    plt.axis('off')
    
    plt.subplot(3, 3, i*3+3)
    plt.imshow(pred_mask, cmap='gray')
    plt.title(f"Predicted Mask\nPred Class: {pred_label}")
    plt.axis('off')
  
  plt.tight_layout()
  plt.show()

if __name__ == '__main__':
  device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
  print(f"Using device: {device}")
  
  # Обучение
  model = train_model()
  
  # Сохранение модели
  torch.save(model.state_dict(), 'object_detector.pth')

  visualize_predictions(model, device)