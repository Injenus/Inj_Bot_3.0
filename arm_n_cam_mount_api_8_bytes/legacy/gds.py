import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json

import dkp

with open('config_arm.json', 'r') as file:
    config = json.load(file)

# Константы (длины звеньев)
l1, l2, l3 = config['length']
ang_range = config['ang_range']  # [[min_a, max_a], [min_b, max_b], [min_c, max_c]]

# Заданные локальные координаты в вертикальной плоскости
x_p, y_p = 200, 250

# Проверка на достижимость
def is_reachable(x, y):
    distance = np.sqrt(x**2 + y**2)
    R_max = l1 + l2 + l3
    R_min = abs(l1 - l2 - l3)
    return R_min <= distance <= R_max

# Проверка на коллизии
def is_collision(angles):
    a, b, c = angles
    points = [
        (0, 0),
        (l1 * np.sin(a), l1 * np.cos(a)),
        (l1 * np.sin(a) + l2 * np.sin(a + b), l1 * np.cos(a) + l2 * np.cos(a + b)),
        (l1 * np.sin(a) + l2 * np.sin(a + b) + l3 * np.sin(a + b + c), l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c))
    ]
    return segments_intersect(points[0], points[1], points[2], points[3])

def segments_intersect(p1, p2, p3, p4):
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (ccw(p1, p2, p3) != ccw(p1, p2, p4))

# Ограничение углов
def constrain_angles(angles):
    constrained = []
    for angle, (min_ang, max_ang) in zip(np.degrees(angles), ang_range):
        angle = (angle + 360) % 360  # Нормализация угла
        if min_ang <= angle <= max_ang:
            constrained.append(np.radians(angle))
        else:
            return None
    return np.array(constrained)

# Функции системы с углами относительно предыдущих звеньев
def f(angles):
    a, b, c = angles
    f1 = l1 * np.sin(a) + l2 * np.sin(a + b) + l3 * np.sin(a + b + c) - x_p
    f2 = l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c) - y_p
    return np.array([f1, f2])

# Метод градиентного спуска с множественными начальными условиями
def gradient_descent_search(max_iter=10000, learning_rate=0.1, tolerance=10):
    best_solution = None
    min_error = float('inf')

    for _ in range(20):  # Несколько попыток с разными начальными условиями
        angles = np.random.uniform(low=[np.radians(a[0]) for a in ang_range],
                                   high=[np.radians(a[1]) for a in ang_range])

        for _ in range(max_iter):
            error = f(angles)
            grad = np.array([
                (f(angles + np.array([1e-5, 0, 0])) - error) / 1e-5,
                (f(angles + np.array([0, 1e-5, 0])) - error) / 1e-5,
                (f(angles + np.array([0, 0, 1e-5])) - error) / 1e-5
            ]).T

            angles -= learning_rate * grad.sum(axis=0)

            if constrain_angles(angles) is None or is_collision(angles):
                continue
                pass

            if np.linalg.norm(error) < tolerance:
                if np.linalg.norm(error) < min_error:
                    min_error = np.linalg.norm(error)
                    best_solution = np.degrees(angles)
                    print('f')
                break

    return best_solution

# Визуализация манипулятора
def plot_manipulator(angles_deg):
    a, b, c = np.radians(angles_deg)
    x0, y0, z0 = 0, 0, 0
    x1 = x0 + l1 * np.sin(a)
    y1 = y0 + l1 * np.cos(a)
    x2 = x1 + l2 * np.sin(a + b)
    y2 = y1 + l2 * np.cos(a + b)
    x3 = x2 + l3 * np.sin(a + b + c)
    y3 = y2 + l3 * np.cos(a + b + c)

    xs = [x0, x1, x2, x3]
    ys = [y0, y1, y2, y3]
    zs = [0, 0, 0, 0]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, marker='o', markersize=8, linewidth=2, color='b')
    ax.set_xlim(-l1-l2-l3, l1+l2+l3)
    ax.set_ylim(-l1-l2-l3, l1+l2+l3)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

if __name__ == '__main__':
    if is_reachable(x_p, y_p):
        solution = gradient_descent_search()
        if solution is not None:
            print(f"Углы решения: {solution}")
            print(f'Реальное положение: {dkp.dkp_2d([l1,l2,l3], solution)}')
            plot_manipulator(solution)
        else:
            print(f"Точка ({x_p}, {y_p}) недостижима.")
    else:
        print(f"Точка ({x_p}, {y_p}) вне зоны досягаемости.")
