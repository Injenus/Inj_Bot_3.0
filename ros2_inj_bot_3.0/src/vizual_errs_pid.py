import matplotlib.pyplot as plt
import numpy as np

# Чтение данных из файла
with open('errs_pid.txt', 'r') as file:
    lines = file.readlines()

# Разделение данных на итерации по пустым строкам
iterations = []
current_iter = []
for line in lines:
    line = line.strip()
    if not line:
        if current_iter:
            iterations.append(current_iter)
            current_iter = []
    else:
        current_iter.append(line)
if current_iter:
    iterations.append(current_iter)

# Обработка каждой итерации
for idx, iter_data in enumerate(iterations, 1):
    side_errors = []
    side_velocities = []
    font_errors = []
    font_velocities = []
    
    # Парсинг данных для side и font
    for line in iter_data:
        if line.startswith('side'):
            parts = line.split()
            error = float(parts[1].strip(','))
            velocity = float(parts[2])
            side_errors.append(error)
            side_velocities.append(velocity)
        elif line.startswith('font'):
            parts = line.split()
            error = float(parts[1].strip(','))
            velocity = float(parts[2])
            font_errors.append(error)
            font_velocities.append(velocity)
    
    # Временные метки (5 мс на шаг)
    time = np.arange(len(side_errors)) * 5  # в миллисекундах
    
    # Построение графиков
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'Итерация {idx}', fontsize=16)
    
    # Ошибка (side)
    axes[0, 0].plot(time, side_errors, 'r-', label='Ошибка')
    axes[0, 0].set_title('Ошибка (side)')
    axes[0, 0].set_xlabel('Время (мс)')
    axes[0, 0].set_ylabel('Ошибка')
    axes[0, 0].grid(True)
    
    # Угловая скорость (side)
    axes[0, 1].plot(time, side_velocities, 'b-', label='Угловая скорость')
    axes[0, 1].set_title('Угловая скорость (side)')
    axes[0, 1].set_xlabel('Время (мс)')
    axes[0, 1].set_ylabel('Скорость')
    axes[0, 1].grid(True)
    
    # Ошибка (font)
    axes[1, 0].plot(time, font_errors, 'g-', label='Ошибка')
    axes[1, 0].set_title('Ошибка (font)')
    axes[1, 0].set_xlabel('Время (мс)')
    axes[1, 0].set_ylabel('Ошибка')
    axes[1, 0].grid(True)
    
    # Угловая скорость (font)
    axes[1, 1].plot(time, font_velocities, 'm-', label='Угловая скорость')
    axes[1, 1].set_title('Угловая скорость (font)')
    axes[1, 1].set_xlabel('Время (мс)')
    axes[1, 1].set_ylabel('Скорость')
    axes[1, 1].grid(True)
    
    # Сохранение в PNG
    plt.tight_layout()
    plt.savefig(f'iteration_{idx}_summary.png')
    plt.close()

print("Графики успешно сохранены в текущую директорию.")