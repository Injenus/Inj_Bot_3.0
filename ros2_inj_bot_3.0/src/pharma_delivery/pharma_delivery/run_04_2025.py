import subprocess
import sys
import time

def run_command(command, description=""):
    print(f"\n\033[1;34m=== Запуск: {description} ===\033[0m")
    try:
        process = subprocess.run(
            command,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print(f"\033[1;32mУспешно: {description}\033[0m")
        return True
    except subprocess.CalledProcessError as e:
        print(f"\033[1;31mОшибка в {description} (код {e.returncode}):\033[0m")
        print(e.stderr)
        return False

if __name__ == "__main__":
    # # 1. Запуск первого узла ROS2
    # node1 = ["ros2", "run", "pharma_delivery", "ver_04_2025"]  # Замените на свои значения
    # if not run_command(node1, "Первый узел ROS2"):
    #     sys.exit(1)
    # 2. Запуск SH-скрипта
    node3 = ["pyhton ", "/home/inj/Inj_Bot_3.0/ros2_inj_bot_3.0/src/pharma_delivery/pharma_delivery/ver_04_2025_bag.py"]  # Замените на свои значения
    if not run_command(node3, "Второй узел ROS2"):
        sys.exit(1)

    # 2. Запуск SH-скрипта
    sh_script = ["/home/inj/Inj_Bot_3.0/ros2_inj_bot_3.0/src/pharma_delivery/pharma_delivery/bag.sh"]  # Путь к скрипту
    if not run_command(sh_script, "SH-скрипт"):
        sys.exit(1)

    # 3. Запуск второго узла ROS2
    node2 = ["ros2", "run", "pharma_delivery", "init_pos"]  # Замените на свои значения
    if not run_command(node2, "Второй узел ROS2"):
        sys.exit(1)

    print("\n\033[1;32m=== Все задачи выполнены! ===\033[0m")