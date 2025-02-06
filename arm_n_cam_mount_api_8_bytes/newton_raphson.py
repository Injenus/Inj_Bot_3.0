import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import differential_evolution
import json
import random
from matplotlib.path import Path

import dkp


# Проверка на достижимость
def is_reachable(x, y, lengths):
    l1,l2,l3 = lengths
    distance = np.sqrt(x**2 + y**2)
    R_max = l1 + l2 + l3
    R_min = 10
    is_r_valid = (R_min <= distance <= R_max)

    polygon = np.array([[-50, 0], [0, 15], [50, 0]])
    path = Path(polygon)
    is_inside_polyg = path.contains_point((x,y), radius=-1e-9)

    return is_r_valid and not is_inside_polyg

def is_collision(angles, lengths):
    a, b, c = angles
    l1,l2,l3 = lengths
    points = [
        (0, 0),
        (l1 * np.sin(a), l1 * np.cos(a)),
        (l1 * np.sin(a) + l2 * np.sin(a + b), l1 * np.cos(a) + l2 * np.cos(a + b)),
        (l1 * np.sin(a) + l2 * np.sin(a + b) + l3 * np.sin(a + b + c), 
         l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c))
    ]
    
    # Проверка всех возможных пар звеньев
    return (
        segments_intersect(points[0], points[1], points[1], points[2]) or  # Звено 1 и Звено 2
        segments_intersect(points[1], points[2], points[2], points[3]) or  # Звено 2 и Звено 3
        segments_intersect(points[0], points[1], points[2], points[3])     # Звено 1 и Звено 3
    )

def segments_intersect(p1, p2, p3, p4):
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
    # Основная проверка на пересечение отрезков
    return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (ccw(p1, p2, p3) != ccw(p1, p2, p4))

# Функции системы с углами относительно предыдущих звеньев
def f(angles, x, y, lengths):
    l1,l2,l3 = lengths
    a, b, c = angles
    f1 = l1 * np.sin(a) + l2 * np.sin(a + b) + l3 * np.sin(a + b + c) - x
    f2 = l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c) - y
    return np.array([f1, f2])

# Якобиан для новых уравнений
def jacobian(lengths, angles):
    l1, l2, l3 = lengths
    a, b, c = angles
    J = np.zeros((2, 3))

    # Частные производные для x
    J[0, 0] = l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c)
    J[0, 1] = l2 * np.cos(a + b) + l3 * np.cos(a + b + c)
    J[0, 2] = l3 * np.cos(a + b + c)

    # Частные производные для y
    J[1, 0] = -l1 * np.sin(a) - l2 * np.sin(a + b) - l3 * np.sin(a + b + c)
    J[1, 1] = -l2 * np.sin(a + b) - l3 * np.sin(a + b + c)
    J[1, 2] = -l3 * np.sin(a + b + c)

    return J

# Метод Ньютона-Рафсона
def newton_raphson(initial_guess, x, y, lengths, tol=0.001, max_iter=1000):
    angles = np.radians(initial_guess)
    for _ in range(max_iter):
        J = jacobian(lengths,angles)
        F = f(angles, x, y, lengths)
        delta = np.linalg.lstsq(J.T @ J, -J.T @ F, rcond=None)[0]
        angles += delta

        # Проверка сходимости
        if np.linalg.norm(delta) < tol:
            break

    return np.degrees(angles)

def is_within_limits(angles, ang_range):
    a, b, c = angles
    a_min, a_max = ang_range[1]
    b_min, b_max = ang_range[2]
    c_min, c_max = ang_range[3]
    return (a_min <= a <= a_max) and (b_min <= b <= b_max) and (c_min <= c <= c_max)


def plot_manipulator(angles_deg):
    total_length = l1 + l2 + l3  # Для автоматического масштабирования

    # Преобразование углов в радианы
    a, b, c = np.radians(angles_deg)

    # Начальная точка (база манипулятора)
    x0, y0, z0 = 0, 0, 0

    # Первая точка (после первого звена)
    x1 = x0 + l1 * np.sin(a)
    y1 = y0 + l1 * np.cos(a)
    z1 = 0

    # Вторая точка (после второго звена)
    x2 = x1 + l2 * np.sin(a+b)
    y2 = y1 + l2 * np.cos(a+b)
    z2 = 0

    # Третья точка (после третьего звена — конец манипулятора)
    x3 = x2 + l3 * np.sin(a+b+c)
    y3 = y2 + l3 * np.cos(a+b+c)
    z3 = 0

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
    ax.set_title('Положение манипулятора с осями координат')

    # Одинаковый масштаб для осей
    ax.set_box_aspect([1, 1, 0.01])  # XY равны, Z сжат для плоского вида

    plt.legend()
    plt.show()

def get_solution(x, y, ang_range, lengths):
    init_ang = [0, 0, 0]
    solution = newton_raphson(init_ang, x, y, lengths)

    if solution is not None and is_within_limits(solution, ang_range) and not is_collision(solution, lengths):
        return solution
    else:
        # Попытки с другими начальными условиями
        attempts = 0
        max_attempts = 100  # Ограничим количество попыток
        while attempts < max_attempts:
            init_ang = [
                random.uniform(*ang_range[1]),
                random.uniform(*ang_range[2]),
                random.uniform(*ang_range[3])
            ]
            solution = newton_raphson(init_ang, x, y, lengths)
            if solution is not None and is_within_limits(solution, ang_range) and not is_collision(solution, lengths):
                return solution
            attempts += 1
    return None, None, None  # Если не удалось найти решение


if __name__ == '__main__':
    with open('config_arm.json', 'r') as file:
        config = json.load(file)
    # Константы (длины звеньев)
    l1, l2, l3 = config['length']
    ang_range = config['ang_range']  # [[min_q, max_q], [min_a, max_a], [min_b, max_b], [min_c, max_c]]
    # Заданные локальные координаты в вертикальной плоскости
    x_p, y_p = 154, 10
    inp = input()
    if inp != 'd':
        try:
            x_p, y_p = map(int, inp.split())
        except:
            print(f'invalid input, def values are using ({x_p}, {y_p})')

    if is_reachable(x_p, y_p, (l1,l2,l3)):
        solution = get_solution(x_p, y_p, ang_range, (l1,l2,l3))
        if solution is not None:
            print(f"Углы решения: {solution}")
            print(f'Итоговое положение схвата: {dkp.dkp_2d([l1, l2, l3], solution)}')
            plot_manipulator(solution)
        else:
            print(f"Точка ({x_p}, {y_p}) недостижима.")
    else:
        print(f"Точка ({x_p}, {y_p}) вне зоны досягаемости.")
