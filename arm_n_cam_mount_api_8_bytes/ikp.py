import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import differential_evolution
import json
import random
import os

import dkp
import newton_raphson

def view(lengths, angles, name='Положение манипулятора с осями координат'):
    l1,l2,l3 = lengths
    q0,q1,q2,q3 = angles

    total_length = sum(lengths)

    x0, y0, z0, r0 = 0, 0, 0, 0

    # Первая точка (после первого звена)
    r1 = r0 + l1*np.sin(q1)
    x1 = x0 + r1 * np.sin(q0)
    y1 = y0 + r1 * np.cos(q0)
    z1 = z0 + l1*np.cos(q1) 

    # Вторая точка (после второго звена)
    r2 = r1 + l2*np.sin(q1+q2)
    x2 = r2 * np.sin(q0)
    y2 = r2 * np.cos(q0)
    z2 = z1 + l2*np.cos(q1+q2)

    # Третья точка (после третьего звена — конец манипулятора)
    r3 = r2 + l3*np.sin(q1+q2+q3)
    x3 = r3 * np.sin(q0)
    y3 = r3 * np.cos(q0)
    z3 = z2 + l3*np.cos(q1+q2+q3)

    print(f'z3 {z3}')
    print(f'l1={np.linalg.norm(np.array([x1,y1,z1]) - np.array([x0,y0,z0]))}')
    print(f'l2={np.linalg.norm(np.array([x2,y2,z2]) - np.array([x1,y1,z1]))}')
    print(f'l3={np.linalg.norm(np.array([x3,y3,z3]) - np.array([x2,y2,z2]))}')

    # Координаты для отрисовки
    xs = [x0, x1, x2, x3]
    ys = [y0, y1, y2, y3]
    zs = [z0, z1, z2, z3]

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

def main(x,y,z, name='def'):
    dir = 'ikp_log'
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
    if newton_raphson.is_reachable(x_p, y_p, (l1,l2,l3)):
        deg_ang = newton_raphson.get_solution(x_p, y_p, ang_range, (l1,l2,l3))
        if all(x is not None for x in deg_ang):
            q1,q2,q3 = np.radians(deg_ang)
            a,b,c = deg_ang
            print(f"Углы в локальной плоскости: {deg_ang}")
            print(f'Угол theta = {np.degrees(theta)}')
            print(f"Реальное положение схвата: {dkp.dkp_3d((l1,l2,l3),(np.degrees(theta),a,b,c))}")
            view((l1, l2, l3), (q0,q1,q2,q3), f'{dir}/{name} x={x}, y={y}, z={z}')
        else:
            print(f"Точка ({x_p}, {y_p}) в локальной плоскости недостижима.")
            with open(f"{dir}/Недостиж {name} x={x}, y={y}, z={z}.txt", "w") as file:
                file.write("")
    else:
        print(f"Точка ({x_p}, {y_p}) в локальной плокости вне зоны досягаемости.")
        with open(f"{dir}/Вне з {name} x={x}, y={y}, z={z}.txt", "w") as file:
                file.write("")


if __name__ == '__main__':
    print()
    with open('config_arm.json', 'r') as file:
        config = json.load(file)
        # Константы (длины звеньев)
        l1, l2, l3 = config['length']
        ang_range = config['ang_range']

    # x,y,z = -20, -150, 50
    # main(x,y,z)

    x = [i for i in range(-330,330,10)]
    y = [i for i in range(-330,330,10)]
    z = [i for i in range(-330,330,10)]

    i = 0
    for x_ in x:
        for y_ in y:
            for z_ in z:
                main(x_,y_,z_, i)
                i += 1

    