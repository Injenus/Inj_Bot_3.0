from PIL import Image, ImageDraw

# Настройки формата А4 и DPI
A4_WIDTH_MM = 210
A4_HEIGHT_MM = 297
DPI = 300
MM_TO_PIXELS = DPI / 25.4  # Пикселей в 1 мм

# Рассчет размеров в пикселях
width_px = int(A4_WIDTH_MM * MM_TO_PIXELS)
height_px = int(A4_HEIGHT_MM * MM_TO_PIXELS)

def generate_type1():
    """Вертикальная полоса по центру"""
    img = Image.new("RGB", (width_px, height_px), "white")
    draw = ImageDraw.Draw(img)
    
    stripe_width = int(40 * MM_TO_PIXELS)
    x0 = (width_px - stripe_width) // 2
    draw.rectangle([x0, 0, x0 + stripe_width, height_px], fill="black")
    
    return img

def generate_type2():
    """L-образная полоса с поворотом 90°"""
    img = Image.new("RGB", (width_px, height_px), "white")
    draw = ImageDraw.Draw(img)
    
    stripe_width = int(40 * MM_TO_PIXELS)
    center_x, center_y = width_px//2, height_px//2
    
    # Вертикальная часть (снизу до центра)
    draw.rectangle([
        center_x - stripe_width//2, 0,
        center_x + stripe_width//2, center_y
    ], fill="black")
    
    # Горизонтальная часть (от центра направо)
    draw.rectangle([
        center_x - stripe_width//2, 
        center_y - stripe_width//2,
        width_px,
        center_y + stripe_width//2
    ], fill="black")
    
    return img

def generate_type3():
    """Вертикальная полоса с перпендикуляром"""
    img = Image.new("RGB", (width_px, height_px), "white")
    draw = ImageDraw.Draw(img)
    
    stripe_width = int(40 * MM_TO_PIXELS)
    center_x, center_y = width_px//2, height_px//2
    
    # Основная вертикальная полоса
    draw.rectangle([
        center_x - stripe_width//2, 0,
        center_x + stripe_width//2, height_px
    ], fill="black")
    
    # Горизонтальный перпендикуляр
    draw.rectangle([
        center_x - stripe_width//2,
        center_y - stripe_width//2,
        width_px,
        center_y + stripe_width//2
    ], fill="black")
    
    return img

def generate_type4():
    """Короткая полоса с перпендикуляром"""
    img = Image.new("RGB", (width_px, height_px), "white")
    draw = ImageDraw.Draw(img)
    
    stripe_width = int(40 * MM_TO_PIXELS)
    center_x, center_y = width_px//2, height_px//2
    
    # Вертикальная полоса (половинной высоты)
    draw.rectangle([
        center_x - stripe_width//2, 
        center_y - height_px//4,
        center_x + stripe_width//2, 
        center_y + height_px//4
    ], fill="black")
    
    # Горизонтальный перпендикуляр
    draw.rectangle([
        center_x - stripe_width//2,
        center_y - stripe_width//2,
        width_px,
        center_y + stripe_width//2
    ], fill="black")
    
    return img

# Генерация и сохранение изображений
generate_type1().save("type1.png")
generate_type2().save("type2.png")
generate_type3().save("type3.png")
generate_type4().save("type4.png")

print("Изображения успешно сгенерированы!")