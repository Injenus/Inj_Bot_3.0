from evdev import InputDevice, ecodes, list_devices
import pygame
import os
import sys
import time

# Конфигурация
AUDIO_FILE = os.path.join('..', 'audio', 'leopold.wav')
DEBUG = True  # Включить подробный вывод отладки

class AudioController:
    def __init__(self, file_path):
        pygame.mixer.init()
        pygame.mixer.music.load(file_path)
        self.paused = False
        self.playing = False

    def start(self):
        pygame.mixer.music.play()
        self.playing = True
        if DEBUG:
            print("[Audio] Воспроизведение начато")

    def toggle_pause(self):
        if self.paused:
            pygame.mixer.music.unpause()
            self.paused = False
            if DEBUG:
                print("[Audio] Воспроизведение возобновлено")
        else:
            pygame.mixer.music.pause()
            self.paused = True
            if DEBUG:
                print("[Audio] Пауза")

    def stop(self):
        pygame.mixer.music.stop()
        self.playing = False
        if DEBUG:
            print("[Audio] Воспроизведение остановлено")

def find_keyboard():
    """Поиск и выбор клавиатуры с подробной отладкой"""
    devices = [InputDevice(path) for path in list_devices()]
    
    if DEBUG:
        print("\nДоступные устройства ввода:")
        for i, dev in enumerate(devices):
            print(f"{i+1}. {dev.name} ({dev.path})")

    for dev in devices:
        try:
            caps = dev.capabilities()
            if ecodes.EV_KEY in caps:
                if DEBUG:
                    print(f"\nПроверка устройства: {dev.name}")
                    print("Поддерживаемые клавиши:", [ecodes.KEY[k] for k in caps[ecodes.EV_KEY]] if caps[ecodes.EV_KEY] else "Нет клавиш")
                
                # Проверяем наличие необходимых клавиш
                required_keys = {ecodes.KEY_A, ecodes.KEY_P, ecodes.KEY_S}
                if required_keys.issubset(caps[ecodes.EV_KEY]):
                    if DEBUG:
                        print(f"Выбрана клавиатура: {dev.name}")
                    return dev
        except Exception as e:
            if DEBUG:
                print(f"Ошибка при проверке {dev.path}: {str(e)}")
            continue

    raise SystemExit("\nОШИБКА: Не найдено подходящей клавиатуры с клавишами A, P и S!")

def main():
    # Проверка файла
    if not os.path.exists(AUDIO_FILE):
        raise FileNotFoundError(f"Аудиофайл не найден: {AUDIO_FILE}")

    # Инициализация аудио
    controller = AudioController(AUDIO_FILE)
    controller.start()

    # Поиск клавиатуры
    keyboard = find_keyboard()

    # Состояние клавиш
    key_states = {
        ecodes.KEY_A: False,
        ecodes.KEY_P: False,
        ecodes.KEY_S: False
    }

    print("\nУправление:")
    print("1. Зажать A + нажать P - Пауза/Продолжение")
    print("2. Зажать A + нажать S - Остановка")
    print("3. Ctrl+C - Выход из программы\n")

    try:
        while True:
            event = keyboard.read_one()
            if event:
                if event.type == ecodes.EV_KEY:
                    code = event.code
                    state = event.value
                    key_name = ecodes.KEY.get(code, 'UNKNOWN')

                    # Отладочная информация
                    if DEBUG:
                        state_desc = {
                            0: "Отпущена",
                            1: "Нажата",
                            2: "Удерживается"
                        }.get(state, "Неизвестное состояние")
                        print(f"[Клавиша] {key_name} ({code}): {state_desc}")

                    # Обновляем состояние клавиш
                    if code in key_states:
                        key_states[code] = state in [1, 2]

                    # Обработка комбинаций
                    if state == 1:  # Только при нажатии
                        if key_states[ecodes.KEY_A]:
                            if code == ecodes.KEY_P:
                                print("\n>>> Пауза/Продолжение")
                                controller.toggle_pause()
                            elif code == ecodes.KEY_S:
                                print("\n>>> Остановка воспроизведения")
                                controller.stop()
                                return

            # Проверка состояния воспроизведения
            if not controller.playing and not controller.paused:
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nПрервано пользователем")
    finally:
        controller.stop()
        if DEBUG:
            print("\nЗавершение работы...")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nКРИТИЧЕСКАЯ ОШИБКА: {str(e)}")
        sys.exit(1)