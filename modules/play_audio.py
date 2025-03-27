from evdev import InputDevice, ecodes, list_devices
import pygame
import os
import sys
import time

# Конфигурация
AUDIO_FILES = [  # Список аудиофайлов для воспроизведения
    os.path.join('..', 'audio', 'TTS_663477 (mp3cut.net).wav'),
    # os.path.join('..', 'audio', 'ACDC_BACK_IN_BLACK.wav')
]
DEBUG = True  # Включить подробный вывод отладки

class AudioController:
    def __init__(self, file_paths):
        pygame.mixer.init()
        self.file_paths = file_paths
        self.current_track = 0
        self.paused = False
        self.playing = False
        self.load(self.current_track)

    def load(self, track_index=0):
        self.current_track = track_index % len(self.file_paths)
        pygame.mixer.music.load(self.file_paths[self.current_track])
        if DEBUG:
            print(f"[Audio] Загружен трек {self.current_track}: {self.file_paths[self.current_track]}")

    def start(self):
        if not self.playing:
            pygame.mixer.music.play()
            self.playing = True
            self.paused = False
            if DEBUG:
                print("[Audio] Воспроизведение начато")
        else:
            pygame.mixer.music.rewind()
            if DEBUG:
                print("[Audio] Перезапуск трека")

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
        self.paused = False
        if DEBUG:
            print("[Audio] Воспроизведение остановлено")

    def next_track(self):
        self.stop()
        self.load(self.current_track + 1)
        self.start()

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
                
                required_keys = {ecodes.KEY_A, ecodes.KEY_P, ecodes.KEY_S, ecodes.KEY_N}
                if required_keys.issubset(caps[ecodes.EV_KEY]):
                    if DEBUG:
                        print(f"Выбрана клавиатура: {dev.name}")
                    return dev
        except Exception as e:
            if DEBUG:
                print(f"Ошибка при проверке {dev.path}: {str(e)}")
            continue

    raise SystemExit("\nОШИБКА: Не найдено подходящей клавиатуры с необходимыми клавишами!")

def main():
    # Проверка файлов
    for file in AUDIO_FILES:
        if not os.path.exists(file):
            raise FileNotFoundError(f"Аудиофайл не найден: {file}")

    # Инициализация аудио
    controller = AudioController(AUDIO_FILES)

    # Поиск клавиатуры
    keyboard = find_keyboard()

    # Состояние клавиш
    key_states = {
        ecodes.KEY_A: False,
        ecodes.KEY_P: False,
        ecodes.KEY_S: False,
        ecodes.KEY_N: False
    }

    print("\nУправление:")
    print("1. Зажать A + нажать P - Пауза/Продолжение")
    print("2. Зажать A + нажать S - Остановка")
    print("3. Зажать A + нажать N - Следующий трек")
    print("4. Зажать A + нажать Q - Начать/перезапустить")
    print("5. Ctrl+C - Выход из программы\n")

    try:
        while True:
            event = keyboard.read_one()
            if event and event.type == ecodes.EV_KEY:
                code = event.code
                state = event.value
                key_name = ecodes.KEY.get(code, 'UNKNOWN')

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
                if state == 1:
                    if key_states[ecodes.KEY_A]:
                        if code == ecodes.KEY_P:
                            print("\n>>> Пауза/Продолжение")
                            controller.toggle_pause()
                        elif code == ecodes.KEY_S:
                            print("\n>>> Остановка воспроизведения")
                            controller.stop()
                        elif code == ecodes.KEY_N:
                            print("\n>>> Следующий трек")
                            controller.next_track()
                        elif code == ecodes.KEY_Q:
                            print("\n>>> Старт/перезапуск")
                            controller.start()

            # Проверка состояния воспроизведения
            if controller.playing and not pygame.mixer.music.get_busy():
                controller.playing = False
                if DEBUG:
                    print("[Audio] Воспроизведение завершено")
                    exit(0)

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