import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Константы (длины звеньев)
l1, l2, l3 = 124, 63, 149

# Заданные координаты
x_target, y_target = 100, 100

# Проверка на достижимость
def is_reachable(x, y):
    distance = np.sqrt(x**2 + y**2)
    R_max = l1 + l2 + l3
    R_min = abs(l1 - l2 - l3)
    return R_min <= distance <= R_max

# Функции системы с углами относительно предыдущих звеньев
def f(angles):
    a, b, c = angles
    f1 = l1 * np.sin(a) + l2 * np.sin(a + b) + l3 * np.sin(a + b + c) - x_target
    f2 = l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c) - y_target
    return np.array([f1, f2])

# Якобиан для новых уравнений
def jacobian(angles):
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
def newton_raphson(initial_guess, tol=1e-4, max_iter=100):
    angles = np.radians(initial_guess)
    for _ in range(max_iter):
        J = jacobian(angles)
        F = f(angles)
        delta = np.linalg.lstsq(J.T @ J, -J.T @ F, rcond=None)[0]
        angles += delta

        # Проверка сходимости
        if np.linalg.norm(delta) < tol:
            break

    # Ограничение углов от -128° до 127°
    #angles = np.clip(angles, np.radians(-128), np.radians(127))
    return np.degrees(angles)

# Основной блок
if is_reachable(x_target, y_target):
    initial_guess = [-90, -45, -30]  # Начальная догадка с отрицательными углами
    angles_deg = newton_raphson(initial_guess)
    print(f"Оптимальные углы (в градусах): a = {angles_deg[0]:.2f}°, b = {angles_deg[1]:.2f}°, c = {angles_deg[2]:.2f}°")
else:
    print(f"Точка ({x_target}, {y_target}) недостижима для манипулятора.")


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
    ax.set_box_aspect([1, 1, 0.1])  # XY равны, Z сжат для плоского вида

    plt.legend()
    plt.show()

plot_manipulator(angles_deg)
#plot_manipulator(np.array([0,45,45]))