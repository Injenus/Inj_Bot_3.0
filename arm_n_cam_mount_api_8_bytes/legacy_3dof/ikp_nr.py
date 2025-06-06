import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import differential_evolution
import json
import os
import time
import gc
import psutil
import sys

import dkp
import newton_raphson

def memory_limit_exceeded(max_mb):
    """Проверяет, превышено ли использование памяти."""
    process = psutil.Process()
    mem_info = process.memory_info()
    return (mem_info.rss / 1024 / 1024) > max_mb  # RSS в мегабайтах

def view(lengths, angles, name='Положение манипулятора с осями координат'):
    total_length = sum(lengths)

    xs,ys,zs = dkp.arm_unit_coords_3d(lengths, angles)

    # Визуализация
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, marker='o', markersize=8, linewidth=2, color='b', label='Манипулятор')

    # Отметка точек
    for i, (x, y, z) in enumerate(zip(xs, ys, zs)):
        ax.text(x, y, z, f'P{i}', fontsize=10)

    # Добавление координатных осей
    axis_length = total_length * 1.2  # Длина осей с небольшим запасом

    # Ось X (красная)
    ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', arrow_length_ratio=0.1)
    ax.text(axis_length, 0, 0, 'X', color='r', fontsize=12)

    # Ось Y (зелёная)
    ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', arrow_length_ratio=0.1)
    ax.text(0, axis_length, 0, 'Y', color='g', fontsize=12)

    # Ось Z (синяя)
    ax.quiver(0, 0, 0, 0, 0, axis_length / 4, color='b', arrow_length_ratio=0.1)
    ax.text(0, 0, axis_length / 4, 'Z', color='b', fontsize=12)

    # Настройка равных масштабов для всех осей
    max_range = total_length * 1.5
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-1, 1)  # Манипулятор плоский, остаётся в одной плоскости

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(name)

    # Одинаковый масштаб для осей
    ax.set_box_aspect([1, 1, 0.01])  # XY равны, Z сжат для плоского вида

    plt.legend()
    #plt.show()
    plt.savefig(f'{name}.jpg')

def limit_angle(theta_deg, range):
    # Ограничение угла
    if range[0] <= theta_deg <= range[1]:
        return np.radians(theta_deg)
    else:
        # Зеркалирование угла
        if theta_deg < range[0]:
            mirrored_angle = theta_deg + 180
        else:  # theta_deg > range[1]
            mirrored_angle = theta_deg - 180

        # Нормализация зеркального угла в пределах [-180, 180]
        mirrored_angle = (mirrored_angle + 180) % 360 - 180

        # Проверка, входит ли зеркальный угол в диапазон
        if range[0] <= mirrored_angle <= range[1]:
            return np.radians(mirrored_angle)
        else:
            # Если снова выходит за диапазон, повторить зеркалирование
            return limit_angle(mirrored_angle, range)
        
def cylindrical_to_cartesian(theta, r, h, degrees=False):
    theta_rad = np.radians(theta) if degrees else theta
    return (r * np.sin(theta_rad), r * np.cos(theta_rad), h)

def main(x,y,z, init_ang, name='def'):
    dir = 'ikp_log_Path_segments_EXPERIMENTAL'
    os.makedirs(dir, exist_ok=True)
    
    theta = (np.arctan2(x,y) + np.pi) % (2 * np.pi) - np.pi
    theta = limit_angle(np.degrees(theta), ang_range[0])
    if np.degrees(theta) < ang_range[0][0] or np.degrees(theta) > ang_range[0][1]:
        print(f'По удивительной причине целевая точка недостижима для угла theta = {np.degrees(theta)}')
        
    y_p = z
    x_p = x / np.sin(theta)

    q0 = theta
    # print(f'В локальной плоскости точка ({x_p}, {y_p})')
    # print(f'Угол базового звена {np.degrees(theta)}')
    sol_time = time.time()
    if newton_raphson.is_reachable(x_p, y_p, (l1,l2,l3)):
        deg_ang = newton_raphson.get_solution(x_p, y_p, ang_range, (l1,l2,l3), q0, init_ang, 5)
        sol_time = time.time() - sol_time
        if all(x is not None for x in deg_ang):
            q1,q2,q3 = np.radians(deg_ang)
            a,b,c = deg_ang
            # print(f"Углы в локальной плоскости: {deg_ang}")
            # print(f'Угол theta = {np.degrees(theta)}')
            r_x, r_y, r_z = dkp.dkp_3d((l1,l2,l3),(np.degrees(theta),a,b,c))
            print(f"Реальное положение схвата: x={r_x}, y={r_y}, z={r_z}")
            print(f'Углы в град. q0={np.degrees(theta)}, q1={a}, q2={b},q3={c}')
            view((l1, l2, l3), (q0,q1,q2,q3), f'{dir}/{name} x={x}, y={y}, z={z}, time={sol_time}')
            return a,b,c
        else:
            print(f"{name} Точка ({x_p}, {y_p}) в локальной плоскости недостижима.")
            with open(f"{dir}/{name} Недостиж x={x}, y={y}, z={z}, time={sol_time}.txt", "w") as file:
                file.write("")
            return -1, -1, -1
    else:
        for k in range(42):
            if newton_raphson.is_reachable(x_p, y_p, (l1,l2,l3)):
                sol_time = time.time() - sol_time
                print(f"{name} Точка ({x_p}, {y_p}) в локальной плокости вне зоны досягаемости.")
                with open(f"{dir}/{name} Вне з x={x}, y={y}, z={z}, time={sol_time}.txt", "w") as file:
                    file.write(f"{name} Точка ({x_p}, {y_p}) наебала детерминированный метод.")
                with open(f'{dir}_log.txt', 'a'):
                    file.write(f'{name} Точка ({x_p}, {y_p}), наёбка произошла с {k+1} попытки.')
                return -2, -2, -2
            
        sol_time = time.time() - sol_time
        print(f"{name} Точка ({x_p}, {y_p}) в локальной плокости вне зоны досягаемости.")
        with open(f"{dir}/{name} Вне з x={x}, y={y}, z={z}, time={sol_time}.txt", "w") as file:
            file.write("")
        return -2, -2, -2


if __name__ == '__main__':
    print()
    with open('config_arm.json', 'r') as file:
        config = json.load(file)
        # Константы (длины звеньев)
        l1, l2, l3 = config['length']
        ang_range = config['ang_range']
        ang_offset = config['offset']

    ang_range = [
        [a - off, b - off] 
        for (a, b), off in zip(ang_range, ang_offset)
    ]

    # x,y,z = 1, 1, 10
    # main(x,y,z, [0,0,0], 'def')
    
    step = 100
    max_r = sum(config['length'])
    q1,q2,q3 = 0, 0, 0
    i = 0
    for x in range(-max_r, max_r, step):
        for y in range(-max_r, max_r, step):
            if np.sqrt(x**2+y**2) > max_r:
                continue
            for z in range(-max_r, max_r, step):
                if abs(x+y+z) > max_r or np.sqrt(x**2+z**2) > max_r or np.sqrt(y**2+z**2) > max_r:
                    continue
                i += 1
                print('\nЦелевое положение схвата: ',f'x={x}, y={y}, z={z}')
                q1_,q2_,q3_ = main(x,y,z, [q1,q2,q3], i)
                if not (q1_==-1 and q2_==-1 and q3_==-1) and not (q1_==-2 and q2_==-2 and q3_==-2):
                    q1, q2, q3 = q1_, q2_, q3_
                if not 1%420:
                    # Проверка памяти
                    if memory_limit_exceeded(15000):
                        print(f"Лимит памяти превышен ({15000} MB). Завершение...")
                        sys.exit(1)
                        
                    gc.collect()


    # x = [i for i in range(-330,330,10)]
    # y = [i for i in range(-330,330,10)]
    # z = [i for i in range(-330,330,10)]
 
    # i = 0
    # for x_ in x:
    #     for y_ in y:
    #         for z_ in z:
    #             if i < 0:
    #                 i += 1
    #                 continue
    #             # q1_,q2_,q3_ = main(x_,y_,z_, [q1,q2,q3], i)
    #             # if not (q1_==-1 and q2_==-1 and q3_==-1) and not (q1_==-2 and q2_==-2 and q3_==-2):
    #             #     q1, q2, q3 = q1_, q2_, q3_
    #             main(x_,y_,z_, None, i)
    #             i += 1
    #             if not (i % 42):
    #                 gc.collect()

    