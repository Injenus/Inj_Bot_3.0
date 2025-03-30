from evdev import InputDevice, ecodes, list_devices
import pygame
import os
import sys
import time

# Конфигурация
AUDIO_FILES = [  
    os.path.join(os.path.expanduser('~'), 'Music', 'Inj_Bot_audio', 'leopold.wav'),
    os.path.join(os.path.expanduser('~'), 'Music', 'Inj_Bot_audio', 'ACDC_BACK_IN_BLACK.wav'),
    os.path.join(os.path.expanduser('~'), 'Music', 'Inj_Bot_audio', 'ACDC_HIGHWAY_TO_HELL.wav'),
    os.path.join(os.path.expanduser('~'), 'Music', 'Inj_Bot_audio', 'ACDC_SHOOT_TO_THRILL.wav'),
    os.path.join(os.path.expanduser('~'), 'Music', 'Inj_Bot_audio', 'EURYTHMICS_SWEET_DREAMS_ARE_MADE_OF_THIS.wav'),
    os.path.join(os.path.expanduser('~'), 'Music', 'Inj_Bot_audio', 'POHUY.wav'),
    os.path.join(os.path.expanduser('~'), 'Music', 'Inj_Bot_audio', 'ACDC_THROUGH_THE_MISTS_OF_TIME.wav'),
    os.path.join(os.path.expanduser('~'), 'Music', 'Inj_Bot_audio', "ACDC_IT'S_A_LONG_WAY_TO_THE_TOP_IF_YOU_WANNA_ROCK'N'ROLL.wav")
]
DEBUG = True  

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

    def previous_track(self):
        self.stop()
        self.load(self.current_track - 1)
        self.start()

def find_keyboard():
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
                
                required_keys = {ecodes.KEY_A, ecodes.KEY_P, ecodes.KEY_S, 
                               ecodes.KEY_N, ecodes.KEY_Q, ecodes.KEY_B, ecodes.KEY_ESC}
                if required_keys.issubset(caps[ecodes.EV_KEY]):
                    if DEBUG:
                        print(f"Выбрана клавиатура: {dev.name}")
                    return dev
                else:
                    if DEBUG:
                        print("Не хватает клавиш:", required_keys - set(caps[ecodes.EV_KEY]))
        except Exception as e:
            if DEBUG:
                print(f"Ошибка при проверке {dev.path}: {str(e)}")
            continue

    raise SystemExit("\nОШИБКА: Не найдено подходящей клавиатуры с необходимыми клавишами!")

def main():
    for file in AUDIO_FILES:
        if not os.path.exists(file):
            raise FileNotFoundError(f"Аудиофайл не найден: {file}")

    controller = AudioController(AUDIO_FILES)
    keyboard = find_keyboard()

    key_states = {
        ecodes.KEY_A: False,
        ecodes.KEY_P: False,
        ecodes.KEY_S: False,
        ecodes.KEY_N: False,
        ecodes.KEY_Q: False,
        ecodes.KEY_B: False,
        ecodes.KEY_ESC: False
    }

    print("\nУправление:")
    print("1. Зажать A + нажать P - Пауза/Продолжение")
    #print("2. Зажать A + нажать S - Остановка")
    print("3. Зажать A + нажать N - Следующий трек")
    print("4. Зажать A + нажать B - Предыдущий трек")
    print("5. Зажать A + нажать Q - Начать/перезапустить")
    print("6. ESC - Немедленный выход из программы\n")

    try:
        while True:
            event = keyboard.read_one()
            if event and event.type == ecodes.EV_KEY:
                code = event.code
                state = event.value

                if DEBUG:
                    key_name = ecodes.KEY.get(code, 'UNKNOWN')
                    state_desc = {
                        0: "Отпущена",
                        1: "Нажата",
                        2: "Удерживается"
                    }.get(state, "Неизвестное состояние")
                    print(f"[Клавиша] {key_name} ({code}): {state_desc}")

                if code in key_states:
                    key_states[code] = state in [1, 2]

                if state == 1:
                    if code == ecodes.KEY_ESC:  # Обработка ESC
                        print("\n>>> Немедленный выход!")
                        raise SystemExit
                    
                    if key_states[ecodes.KEY_A]:
                        if code == ecodes.KEY_P:
                            print("\n>>> Пауза/Продолжение")
                            controller.toggle_pause()
                        # elif code == ecodes.KEY_S:
                        #     print("\n>>> Остановка воспроизведения")
                        #     controller.stop()
                        elif code == ecodes.KEY_N:
                            print("\n>>> Следующий трек")
                            controller.next_track()
                        elif code == ecodes.KEY_B:
                            print("\n>>> Предыдущий трек")
                            controller.previous_track()
                        elif code == ecodes.KEY_Q:
                            print("\n>>> Старт/перезапуск")
                            controller.start()

            time.sleep(0.01)

    except SystemExit:
        print("\nЗавершение работы по запросу пользователя")
    finally:
        controller.stop()
        if DEBUG:
            print("\nОчистка ресурсов...")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nКРИТИЧЕСКАЯ ОШИБКА: {str(e)}")
        sys.exit(1)