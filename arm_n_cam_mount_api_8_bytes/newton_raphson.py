import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import differential_evolution
import json
import random
from matplotlib.path import Path
import bisect
import copy

import dkp

random.seed(42)

loaded_data = np.load('valid_area_data.npz')
# Извлекаем основной массив в список
main_loaded = loaded_data['main'].tolist()
# Восстанавливаем список массивов, сохраняя порядок
arrays_loaded = [loaded_data[f'array_{i}'] for i in range(len(main_loaded))]


def find_closest_index(arr, target):
    # Используем bisect_left для нахождения позиции, куда можно вставить target
    pos = bisect.bisect_left(arr, target)
    
    # Если target меньше всех элементов в массиве, возвращаем индекс первого элемента
    if pos == 0:
        return 0
    # Если target больше всех элементов в массиве, возвращаем индекс последнего элемента
    if pos == len(arr):
        return len(arr) - 1
    
    # Сравниваем элемент на позиции pos и pos-1, чтобы найти ближайший
    before = arr[pos - 1]
    after = arr[pos]
    if after - target < target - before:
        return pos
    else:
        return pos - 1

# Проверка на достижимость
def is_reachable(x, y, lengths):
    l1,l2,l3 = lengths
    distance = np.sqrt(x**2 + y**2)
    R_max = l1 + l2 + l3
    R_min = max(0, l1 - l2 - l3, l2 - l1 - l3, l3 - l1 - l2)
    is_r_valid = (R_min <= distance <= R_max)

    is_inside_polyg = is_inside_polygon(x,y)

    return is_r_valid and not is_inside_polyg

def is_inside_polygon(x,y,path=Path(np.array([[-np.inf, -np.inf], [-np.inf, -170], [-210, -170], [-210, 0], [0, 0], [210, 0], [210, -170], [np.inf, -170], [np.inf, -np.inf]]))):
    return path.contains_point((x,y), radius=-1e-9)

def is_segment_intersects_polygon(x1,y1,x2,y2, path):
    # Получаем вершины полигона
    vertices = path.vertices
    n = len(vertices)

    # Функция проверки пересечения двух отрезков
    def segments_intersect(a, b, c, d):
        def ccw(p1, p2, p3):
            return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])
        return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)

    # Проверяем пересечения с каждым ребром полигона
    for i in range(n - 1):
        if segments_intersect((x1, y1), (x2, y2), vertices[i], vertices[i + 1]):
            return True
    return False

def is_collision(angles, lengths):
    a, b, c = np.deg2rad(angles)
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


def f(angles, x_target, y_target, lengths):
    l1, l2, l3 = lengths
    a, b, c = angles  # Углы в радианах
    
    # Координаты по вашей системе:
    # x = сумма проекций на "горизонталь" (ось X) через sin(θ)
    # y = сумма проекций на "вертикаль" (ось Y) через cos(θ)
    x = l1 * np.sin(a) + l2 * np.sin(a + b) + l3 * np.sin(a + b + c)
    y = l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c)
    
    # Невязка между текущими координатами и целевыми
    f1 = x - x_target
    f2 = y - y_target
    
    return np.array([f1, f2])

def jacobian(lengths, angles):
    l1, l2, l3 = lengths
    a, b, c = angles  # Углы в радианах
    J = np.zeros((2, 3))  # Матрица 2x3
    
    # Производные для f1 (по x):
    # df1/da = l1*cos(a) + l2*cos(a+b) + l3*cos(a+b+c)
    # df1/db = l2*cos(a+b) + l3*cos(a+b+c)
    # df1/dc = l3*cos(a+b+c)
    J[1, 0] = l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c)
    J[1, 1] = l2 * np.cos(a + b) + l3 * np.cos(a + b + c)
    J[1, 2] = l3 * np.cos(a + b + c)
    
    # Производные для f2 (по y):
    # df2/da = -l1*sin(a) - l2*sin(a+b) - l3*sin(a+b+c)
    # df2/db = -l2*sin(a+b) - l3*sin(a+b+c)
    # df2/dc = -l3*sin(a+b+c)
    J[0, 0] = -l1 * np.sin(a) - l2 * np.sin(a + b) - l3 * np.sin(a + b + c)
    J[0, 1] = -l2 * np.sin(a + b) - l3 * np.sin(a + b + c)
    J[0, 2] = -l3 * np.sin(a + b + c)
    
    return J

# Метод Ньютона-Рафсона
# def newton_raphson(initial_guess, x, y, lengths, ang_range, tol=1.0, max_iter=100000):
#     angles = np.radians(initial_guess)
#     lower_bounds = np.radians([ang[0] for ang in ang_range[1:]])
#     upper_bounds = np.radians([ang[1] for ang in ang_range[1:]])
#     for _ in range(max_iter):
#         J = jacobian(lengths, angles)
#         F = f(angles, x, y, lengths)
        
#         # Решение с псевдоинверсией
#         delta = -np.linalg.pinv(J) @ F
#         angles = np.clip(angles + delta, lower_bounds, upper_bounds)

#         # Проверка сходимости
#         if np.linalg.norm(delta) <= tol and np.linalg.norm(F) <= tol:
#             break
    
#     solution = np.degrees(angles).tolist()
#     solution[0] = np.clip(solution[0], ang_range[1][0], ang_range[1][1])
#     solution[1] = np.clip(solution[1], ang_range[2][0], ang_range[2][1])
#     solution[2] = np.clip(solution[2], ang_range[3][0], ang_range[3][1])

#     return solution

# def newton_raphson(initial_guess, x_target, y_target, lengths, ang_range, tol=0.5, max_iter=100000):
#     angles = np.radians(initial_guess)  # Градусы -> радианы
#     lower_bounds = np.radians([ang[0] for ang in ang_range[1:]])
#     upper_bounds = np.radians([ang[1] for ang in ang_range[1:]])
    
#     for _ in range(max_iter):
#         J = jacobian(lengths, angles)
#         F = f(angles, x_target, y_target, lengths)
        
#         # Решение методом Гаусса-Ньютона
#         delta = np.linalg.lstsq(J, -F, rcond=None)[0]
#         angles += delta
        
#         # Ограничение углов в процессе итераций
#         angles = np.clip(angles, lower_bounds, upper_bounds)
        
#         # Проверка сходимости
#         if np.linalg.norm(delta) < tol and np.linalg.norm(F) < tol:
#             break
    
#     return np.degrees(angles).tolist()

def newton_raphson(initial_guess, x_target, y_target, lengths, ang_range, tol=1e-1, max_iter=10000, lambda_=0.01):
    angles = np.radians(initial_guess)
    lower_bounds = np.radians([ang[0] for ang in ang_range[1:]])
    upper_bounds = np.radians([ang[1] for ang in ang_range[1:]])
    
    for _ in range(max_iter):
        J = jacobian(lengths, angles)
        F = f(angles, x_target, y_target, lengths)
        
        # Решение методом Левенберга-Марквардта: (J^T J + λI) δ = -J^T F
        I = np.eye(3)
        delta = np.linalg.lstsq(J.T @ J + lambda_ * I, -J.T @ F, rcond=None)[0]
        
        # Линейный поиск с адаптивным шагом
        alpha = 1.0
        for _ in range(10):  # Максимум 10 попыток шага
            new_angles = angles + alpha * delta
            new_angles = np.clip(new_angles, lower_bounds, upper_bounds)
            new_F = f(new_angles, x_target, y_target, lengths)
            
            if np.linalg.norm(new_F) < np.linalg.norm(F):
                angles = new_angles
                lambda_ *= 0.5  # Уменьшаем λ при успешном шаге
                break
            else:
                alpha *= 0.5  # Уменьшаем шаг
                lambda_ *= 2.0  # Увеличиваем λ
        else:
            break  # Не удалось найти подходящий шаг
        
        # Критерий остановки
        if (np.linalg.norm(F) < tol and 
            np.linalg.norm(delta) < tol and 
            np.linalg.norm(alpha * delta) < 0.1 * tol):
            break
            
    return np.degrees(angles).tolist()

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

def get_solution(x, y, ang_range, lengths, q0=None, init_ang=None, range_koeff=4):
    if init_ang is None:
        init_ang = [
                random.uniform(*(x/range_koeff for x in ang_range[1])),
                random.uniform(*(x/range_koeff for x in ang_range[2])),
                random.uniform(*(x/range_koeff for x in ang_range[3]))
            ]
    solution = newton_raphson(init_ang, x, y, lengths, ang_range)

    if solution is not None and not is_collision(solution, lengths):
        return solution
    else:
        # Попытки с другими начальными условиями
        attempts = 0
        max_attempts = 100  # Ограничим количество попыток
        while attempts < max_attempts:
            init_ang = [
                random.uniform(*(x for x in [init_ang[0]*(1-1/range_koeff),init_ang[0]*(1+1/range_koeff)])),
                random.uniform(*(x for x in [init_ang[1]*(1-1/range_koeff),init_ang[1]*(1+1/range_koeff)])),
                random.uniform(*(x for x in [init_ang[2]*(1-1/range_koeff),init_ang[2]*(1+1/range_koeff)]))
            ]
            solution = newton_raphson(init_ang, x, y, lengths, ang_range)
            if solution is not None and not is_collision(solution, lengths):
                valid = True

                # это старый способ, учитывающий только вхождение концов звеньев внутрь полигона
                # xs,ys,zs = dkp.arm_unit_coords_3d(lengths, [q0]+solution)
                # for i in range(1, len(xs)):
                #     if is_inside_polygon((xs[i]**2+ys[i]**2)**0.5, zs[i]):
                #         valid = False
                #         break


                # actual_surface_idx = find_closest_index(main_loaded, q0)
                # xs, ys = dkp.arm_unit_coord_2d(lengths, solution)
                # for i in range(len(xs)-1):
                #     if is_segment_intersects_polygon(xs[i], ys[i], xs[i+1], ys[i+1],
                #                                   Path(arrays_loaded[actual_surface_idx])):
                #         valid = False
                #         break

                if valid:
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
