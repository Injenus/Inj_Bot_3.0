import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import differential_evolution
import json
import os
import time
import gc

import dkp
import newton_raphson

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

def main(x,y,z, init_ang, name='def'):
    dir = 'ikp_log_Path_segments_1000_prev'
    os.makedirs(dir, exist_ok=True)
    
    theta = (np.arctan2(x,y) + np.pi) % (2 * np.pi) - np.pi
    theta = limit_angle(np.degrees(theta), ang_range[0])
    if np.degrees(theta) < ang_range[0][0] or np.degrees(theta) > ang_range[0][1]:
        print(f'По удивительной причине целевая точка недостижима для угла theta = {np.degrees(theta)}')
        
    y_p = z
    x_p = x / np.sin(theta)

    q0 = theta
    print(f'В локальной плоскости точка ({x_p}, {y_p})')
    print(f'Угол базового звена {np.degrees(theta)}')
    sol_time = time.time()
    if newton_raphson.is_reachable(x_p, y_p, (l1,l2,l3)):
        deg_ang = newton_raphson.get_solution(x_p, y_p, ang_range, (l1,l2,l3), q0, init_ang)
        sol_time = time.time() - sol_time
        if all(x is not None for x in deg_ang):
            q1,q2,q3 = np.radians(deg_ang)
            a,b,c = deg_ang
            print(f"Углы в локальной плоскости: {deg_ang}")
            print(f'Угол theta = {np.degrees(theta)}')
            print(f"Реальное положение схвата: {dkp.dkp_3d((l1,l2,l3),(np.degrees(theta),a,b,c))}")
            view((l1, l2, l3), (q0,q1,q2,q3), f'{dir}/{name} x={x}, y={y}, z={z}, time={sol_time}')
            return q1,q2,q3
        else:
            print(f"Точка ({x_p}, {y_p}) в локальной плоскости недостижима.")
            with open(f"{dir}/Недостиж {name} x={x}, y={y}, z={z}, time={sol_time}.txt", "w") as file:
                file.write("")
            return None, None, None
    else:
        sol_time = time.time() - sol_time
        print(f"Точка ({x_p}, {y_p}) в локальной плокости вне зоны досягаемости.")
        with open(f"{dir}/Вне з {name} x={x}, y={y}, z={z}, time={sol_time}.txt", "w") as file:
            file.write("")
        return None, None, None


if __name__ == '__main__':
    print()
    with open('config_arm.json', 'r') as file:
        config = json.load(file)
        # Константы (длины звеньев)
        l1, l2, l3 = config['length']
        ang_range = config['ang_range']

    x,y,z = 1, 1, 10
    main(x,y,z, [0,0,0], 'def')

    x = [i for i in range(-330,330,10)]
    y = [i for i in range(-330,330,10)]
    z = [i for i in range(-330,330,10)]

    q1,q2,q3 = 0,0,0
    i = 0
    for x_ in x:
        for y_ in y:
            for z_ in z:
                if i < 193800:
                    i += 1
                    continue
                q1_,q2_,q3_ = main(x_,y_,z_, [q1,q2,q3], i)
                if q1_ is not None:
                    q1, q2, q3 = q1_, q2_, q3_
                i += 1
                if not (i % 100):
                    gc.collect()

    