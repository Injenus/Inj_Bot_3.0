import matplotlib.pyplot as plt

def parse_blocks(filename):
    """Чтение файла и разделение на блоки данных по пустым строкам"""
    blocks = []
    current_block = []
    
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:  # Найдена пустая строка
                if current_block:  # Сохраняем текущий блок
                    blocks.append(current_block)
                    current_block = []
            else:
                current_block.append(line)
    
    if current_block:  # Добавляем последний блок
        blocks.append(current_block)
    
    return blocks

def process_block(block, iteration):
    """Обработка одного блока данных и построение графиков"""
    side_errors = []
    side_velocities = []
    font_errors = []
    font_velocities = []
    time_ms = []
    
    for i, line in enumerate(block):
        if line.startswith('side'):
            try:
                _, values = line.split()
                error, velocity = map(float, values.split(','))
                side_errors.append(error)
                side_velocities.append(velocity)
            except Exception as e:
                print(f"Ошибка в строке {i} блока {iteration}: {e}")
        
        elif line.startswith('font'):
            try:
                _, values = line.split()
                error, velocity = map(float, values.split(','))
                font_errors.append(error)
                font_velocities.append(velocity)
            except Exception as e:
                print(f"Ошибка в строке {i} блока {iteration}: {e}")
    
    # Создаем временную шкалу (5 мс на каждую пару side-font)
    time_ms = [i * 5 for i in range(len(side_errors))]
    
    # Построение графиков для текущего блока
    if side_errors:
        plt.figure(figsize=(10, 5))
        plt.plot(time_ms, side_errors, 'b-o', markersize=4)
        plt.title(f'Ошибка (side) - Итерация {iteration}')
        plt.xlabel('Время, мс')
        plt.ylabel('Ошибка')
        plt.grid(True)
        plt.savefig(f'side_error_{iteration}.png')
        plt.close()

    if side_velocities:
        plt.figure(figsize=(10, 5))
        plt.plot(time_ms, side_velocities, 'r-o', markersize=4)
        plt.title(f'Угловая скорость (side) - Итерация {iteration}')
        plt.xlabel('Время, мс')
        plt.ylabel('Скорость')
        plt.grid(True)
        plt.savefig(f'side_velocity_{iteration}.png')
        plt.close()

    if font_errors:
        plt.figure(figsize=(10, 5))
        plt.plot(time_ms, font_errors, 'g-o', markersize=4)
        plt.title(f'Ошибка (font) - Итерация {iteration}')
        plt.xlabel('Время, мс')
        plt.ylabel('Ошибка')
        plt.grid(True)
        plt.savefig(f'font_error_{iteration}.png')
        plt.close()

    if font_velocities:
        plt.figure(figsize=(10, 5))
        plt.plot(time_ms, font_velocities, 'm-o', markersize=4)
        plt.title(f'Угловая скорость (font) - Итерация {iteration}')
        plt.xlabel('Время, мс')
        plt.ylabel('Скорость')
        plt.grid(True)
        plt.savefig(f'font_velocity_{iteration}.png')
        plt.close()

# Основная логика выполнения
filename = 'errs_pid.txt'
blocks = parse_blocks(filename)

if not blocks:
    print("Файл не содержит данных!")
else:
    for i, block in enumerate(blocks, 1):
        print(f"Обработка блока {i} ({len(block)} строк)")
        process_block(block, i)
    
    print(f"\nСоздано графиков: {4 * len(blocks)}")
    print("Имена файлов:")
    print(*[f"side_error_{i}.png, side_velocity_{i}.png,\nfont_error_{i}.png, font_velocity_{i}.png" 
          for i in range(1, len(blocks)+1)], sep='\n')