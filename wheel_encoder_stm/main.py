from machine import Pin, UART, Timer
import struct

# Настройка пинов энкодеров
ENCODERS = [
    {"a": Pin('PA0', Pin.IN), "b": Pin('PA1', Pin.IN)},  # Левый передний
    {"a": Pin('PA2', Pin.IN), "b": Pin('PA3', Pin.IN)},  # Левый задний
    {"a": Pin('PA4', Pin.IN), "b": Pin('PA5', Pin.IN)},  # Правый передний
    {"a": Pin('PA6', Pin.IN), "b": Pin('PA7', Pin.IN)},  # Правый задний
]

# Настройка UART (USART1 на PA9 и PA10)
uart = UART(1, baudrate=115200, tx=Pin('PA9'), rx=Pin('PA10'))

# Глобальные переменные
encoder_positions = [0, 0, 0, 0]  # Положение каждого энкодера
encoder_speeds = [0.0, 0.0, 0.0, 0.0]  # Скорости (в RPM) для каждого энкодера
last_states = [0, 0, 0, 0]  # Последние состояния каналов A и B для каждого энкодера
STEPS_PER_REV = 234.3  # Шагов на оборот
UPDATE_INTERVAL = 50  # Интервал обновления в мс

# Обработчики прерываний для каждого энкодера
def encoder_callback(index):
    global encoder_positions, last_states

    a = ENCODERS[index]["a"].value()
    b = ENCODERS[index]["b"].value()
    state = (a << 1) | b

    # Определяем направление вращения
    if state != last_states[index]:
        if (last_states[index] == 0b00 and state == 0b01) or \
           (last_states[index] == 0b01 and state == 0b11) or \
           (last_states[index] == 0b11 and state == 0b10) or \
           (last_states[index] == 0b10 and state == 0b00):
            encoder_positions[index] += 1
        else:
            encoder_positions[index] -= 1
        last_states[index] = state

# Настраиваем прерывания
for i, enc in enumerate(ENCODERS):
    enc["a"].irq(lambda pin, idx=i: encoder_callback(idx), Pin.IRQ_RISING | Pin.IRQ_FALLING)
    enc["b"].irq(lambda pin, idx=i: encoder_callback(idx), Pin.IRQ_RISING | Pin.IRQ_FALLING)

# Таймер для вычисления скорости и отправки данных
def calculate_speeds(timer):
    global encoder_positions, encoder_speeds

    # Копируем данные из глобальных переменных
    positions = encoder_positions[:]
    encoder_positions = [0, 0, 0, 0]

    for i in range(4):
        # Вычисляем скорость (в RPM)
        encoder_speeds[i] = (positions[i] / STEPS_PER_REV) * (60000.0 / UPDATE_INTERVAL)

    # Формируем сообщение
    message = bytearray()
    message.append(ord('S'))  # Добавляем символ S
    for speed in encoder_speeds:
        message.extend(struct.pack('f', speed))  # Добавляем 4 байта для каждого значения float

    # Вычисляем контрольную сумму Fletcher-16
    checksum = fletcher16(message)
    message.extend(struct.pack('H', checksum))  # Добавляем 2 байта контрольной суммы

    # Отправляем сообщение по UART
    uart.write(message)

# Функция вычисления Fletcher-16
def fletcher16(data):
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1

# Запускаем таймер для периодических расчётов
timer = Timer()
timer.init(period=UPDATE_INTERVAL, mode=Timer.PERIODIC, callback=calculate_speeds)
