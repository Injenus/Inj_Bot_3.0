import subprocess
import sys
import time
import os
import atexit
import signal

# Глобальные переменные для хранения процессов
pre_script_process = None
ver_script_process = None

def run_command(command, description=""):
    print(f"\n\033[1;34m=== Запуск: {description} ===\033[0m")
    try:
        # Перенаправляем вывод в реальном времени
        process = subprocess.run(
            command,
            check=True,
            text=True,
            stdout=sys.stdout,  # Вывод прямо в консоль
            stderr=sys.stderr,
            bufsize=1,          # Линейная буферизация
            universal_newlines=True
        )
        print(f"\033[1;32mУспешно: {description}\033[0m")
        return True
    except subprocess.CalledProcessError as e:
        print(f"\033[1;31mОшибка в {description} (код {e.returncode}):\033[0m")
        sys.exit(1)

def run_background_process(command, description=""):
    print(f"\n\033[1;35m=== Фоновый запуск: {description} ===\033[0m")
    return subprocess.Popen(
        command,
        stdout=sys.stdout,
        stderr=sys.stderr,
        bufsize=1,
        universal_newlines=True,
        text=True
    )

def kill_scripts():
    global pre_script_process, ver_script_process
    
    processes = [
        (pre_script_process, "Предварительный скрипт"),
        (ver_script_process, "Python скрипт")
    ]
    
    for proc, name in processes:
        if proc and proc.poll() is None:
            try:
                print(f"\n\033[1;33mЗавершаем {name} (PID: {proc.pid})\033[0m")
                proc.terminate()
                time.sleep(1)
                if proc.poll() is None:
                    proc.kill()
                proc.wait()
                print(f"\033[1;32m{name} успешно завершен\033[0m")
            except Exception as e:
                print(f"\033[1;31mОшибка при завершении {name}: {e}\033[0m")

def signal_handler(sig, frame):
    kill_scripts()
    sys.exit(0)

if __name__ == "__main__":
    # Регистрация обработчиков
    atexit.register(kill_scripts)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # 1. Запуск предварительного скрипта
        pre_script_path = os.path.expanduser(
            "~/Inj_Bot_3.0/ros2_inj_bot_3.0/src/for_pharma_ver_04_2025.sh"
        )
        pre_script_process = run_background_process(
            [pre_script_path],
            "Предварительный скрипт"
        )
        print(f"PID: {pre_script_process.pid}")

        # 2. Запуск ver_04_2025.py с выводом логов
        ver_script_path = "/home/inj/Inj_Bot_3.0/ros2_inj_bot_3.0/src/pharma_delivery/pharma_delivery/ver_04_2025.py"
        ver_script_process = run_background_process(
            ["python3", "-u", ver_script_path],  # -u для unbuffered output
            "Основной Python скрипт"
        )
        print(f"PID: {ver_script_process.pid}")

        time.sleep(10)

        # Ожидание завершения Python скрипта
        ver_script_process.wait()
        print("\033[1;36m\n=== Python скрипт завершил работу ===\033[0m")

        # 3. Завершение предварительного скрипта
        kill_scripts()

        # 4. Запуск bag.sh
        run_command([
            "/home/inj/Inj_Bot_3.0/ros2_inj_bot_3.0/src/pharma_delivery/pharma_delivery/bag.sh"
        ], "SH-скрипт")

        # 5. Запуск ROS2 узла
        run_command(
            ["ros2", "run", "pharma_delivery", "init_pos"],
            "ROS2 узел init_pos"
        )

        print("\n\033[1;32m=== Все задачи выполнены! ===\033[0m")

    except Exception as e:
        print(f"\033[1;31mКритическая ошибка: {e}\033[0m")
        sys.exit(1)