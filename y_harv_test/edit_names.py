import os

def rename_files(directory='.'):
    for filename in os.listdir(directory):
        # Получаем полный путь к файлу
        old_path = os.path.join(directory, filename)
        
        # Пропускаем подпапки
        if not os.path.isfile(old_path):
            continue
        
        # Разделяем имя и расширение
        name, ext = os.path.splitext(filename)
        
        # Проверяем, заканчивается ли имя на '_res'
        if name.endswith('_res'):
            # Удаляем '_res' из имени
            new_name = name[:-4] + ext
            new_path = os.path.join(directory, new_name)
            
            # Переименовываем файл
            os.rename(old_path, new_path)
            print(f'Переименован: {filename} -> {new_name}')

if __name__ == '__main__':
    # Укажите путь к вашей папке (по умолчанию — текущая папка)
    rename_files()