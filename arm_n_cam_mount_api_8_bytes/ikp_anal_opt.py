import numpy as np
from math import atan2, sqrt
import json
from collections import defaultdict
import numpy as np

class SpatialHash:
    def __init__(self, cell_size=0.2):  # Размер ячейки ~20 см
        self.cell_size = cell_size
        self.grid = defaultdict(list)
        
    def update(self, points):
        """Обновляет хеш-таблицу для заданных точек (Nx3 numpy array)"""
        self.grid.clear()
        for idx, point in enumerate(points):
            cell = tuple((point // self.cell_size).astype(int))
            self.grid[cell].append(idx)
            
    def get_collision_candidates(self):
        """Возвращает пары индексов потенциально пересекающихся звеньев"""
        candidates = set()
        for cell, indices in self.grid.items():
            for i in indices:
                for j in indices:
                    if i < j and abs(i - j) > 1:  # Исключаем смежные звенья
                        candidates.add((i, j))
        return candidates

class InverseKinematicsSolver:
    def __init__(self, link_lengths, ang_range):
        self.L = link_lengths  # Длины звеньев [L1, L2, L3]
        self.prev_angles = None  # Предыдущие углы
        self.angles_range = ang_range

        self.spatial_hash = SpatialHash(cell_size=0.3)
        self.collision_margin = 0.05  # 5 см безопасный зазор

    def compute_link_positions(self, angles):
        """Возвращает 3D позиции всех суставов"""
        q0, q1, q2, q3 = angles
        L1, L2, L3 = self.L
        
        # Базовый поворот
        R = np.array([
            [np.cos(q0), -np.sin(q0), 0],
            [np.sin(q0), np.cos(q0), 0],
            [0, 0, 1]
        ])
        
        # Позиции в локальной системе
        points_local = np.array([
            [0, 0, 0],
            [L1*np.cos(q1), 0, L1*np.sin(q1)],
            [L1*np.cos(q1) + L2*np.cos(q1+q2), 
             0, 
             L1*np.sin(q1) + L2*np.sin(q1+q2)],
            [L1*np.cos(q1) + L2*np.cos(q1+q2) + L3*np.cos(q1+q2+q3),
             0,
             L1*np.sin(q1) + L2*np.sin(q1+q2) + L3*np.sin(q1+q2+q3)]
        ])
        
        # Преобразование в глобальную систему
        return (R @ points_local.T).T
        
    def forward_kinematics(self, angles):
        """Прямая кинематика для 4-DOF манипулятора"""
        q0, q1, q2, q3 = angles
        L1, L2, L3 = self.L
        
        # Базовый поворот
        R_base = np.array([
            [np.cos(q0), -np.sin(q0), 0],
            [np.sin(q0), np.cos(q0), 0],
            [0, 0, 1]
        ])
        
        # Координаты в плоскости манипулятора
        x_plane = L1*np.cos(q1) + L2*np.cos(q1+q2) + L3*np.cos(q1+q2+q3)
        z_plane = L1*np.sin(q1) + L2*np.sin(q1+q2) + L3*np.sin(q1+q2+q3)
        
        # Преобразование в глобальную систему
        pos = R_base @ np.array([x_plane, 0, z_plane])
        return pos
    
    def is_within_limits(self, ang_range):
        a, b, c = ang_range
        a_min, a_max = ang_range[1]
        b_min, b_max = ang_range[2]
        c_min, c_max = ang_range[3]
        return (a_min <= a <= a_max) and (b_min <= b <= b_max) and (c_min <= c <= c_max)

    def is_collision(self, angles_deg):
        a, b, c = angles_deg
        l1,l2,l3 = [l*1000 for l in self.L]
        points = [
            (0, 0),
            (l1 * np.sin(a), l1 * np.cos(a)),
            (l1 * np.sin(a) + l2 * np.sin(a + b), l1 * np.cos(a) + l2 * np.cos(a + b)),
            (l1 * np.sin(a) + l2 * np.sin(a + b) + l3 * np.sin(a + b + c), 
            l1 * np.cos(a) + l2 * np.cos(a + b) + l3 * np.cos(a + b + c))
        ]
        def segments_intersect(p1, p2, p3, p4):
            def ccw(A, B, C):
                return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
            # Основная проверка на пересечение отрезков
            return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (ccw(p1, p2, p3) != ccw(p1, p2, p4))
                
        # Проверка всех возможных пар звеньев
        return (
            segments_intersect(points[0], points[1], points[1], points[2]) or  # Звено 1 и Звено 2
            segments_intersect(points[1], points[2], points[2], points[3]) or  # Звено 2 и Звено 3
            segments_intersect(points[0], points[1], points[2], points[3])     # Звено 1 и Звено 3
        )

    
    def analytical_base_rotation(self, target):
        """Аналитический расчет угла базового поворота"""
        return atan2(target[1], target[0])
    
    
    def solve_planar_ik(self, target_plane, prev_angles_plane):
        """Решение для 3-DOF плоского манипулятора"""
        L1, L2, L3 = self.L
        x, z = target_plane
        eps = 1e-4
        max_iter = 1000
        
        # Начальное приближение из предыдущего решения
        q1, q2, q3 = prev_angles_plane

        for _ in range(max_iter):
            # Вычисление позиций звеньев
            full_angles = np.array([q0_temp, q1, q2, q3])
            link_positions = self.compute_link_positions(full_angles)
            
            # Обновление пространственного хеша
            self.spatial_hash.update(link_positions)
            collision_pairs = self.spatial_hash.get_collision_candidates()
            
            # Расчет штрафа за коллизии
            collision_grad = np.zeros(3)
            for i, j in collision_pairs:
                grad = self.collision_gradient(
                    link_positions[i], 
                    link_positions[j],
                    [q1, q2, q3]
                )
                collision_grad += grad
                
            # Модифицированный градиент
            dq = dq_target + 0.5 * collision_grad  # Коэффициент эмпирически

            # Вычисление текущей позиции
            current_x = L1*np.cos(q1) + L2*np.cos(q1+q2) + L3*np.cos(q1+q2+q3)
            current_z = L1*np.sin(q1) + L2*np.sin(q1+q2) + L3*np.sin(q1+q2+q3)
            error = np.array([x - current_x, z - current_z])
            
            if np.linalg.norm(error) < eps:
                break
            
            # Аналитический якобиан
            J = np.array([
                [-L1*np.sin(q1) - L2*np.sin(q1+q2) - L3*np.sin(q1+q2+q3),
                 -L2*np.sin(q1+q2) - L3*np.sin(q1+q2+q3),
                 -L3*np.sin(q1+q2+q3)],
                 
                [L1*np.cos(q1) + L2*np.cos(q1+q2) + L3*np.cos(q1+q2+q3),
                 L2*np.cos(q1+q2) + L3*np.cos(q1+q2+q3),
                 L3*np.cos(q1+q2+q3)]
            ])
            
            # Регуляризованное псевдообращение
            lambda_reg = 0.1
            dq = np.linalg.pinv(J.T @ J + lambda_reg*np.eye(3)) @ J.T @ error
            
            q1 += dq[0]
            q2 += dq[1]
            q3 += dq[2]

        # После вычисления q1, q2, q3 добавляем:
        solution_deg = np.rad2deg([q1, q2, q3])
            
        # Проверка коллизий
        if self.is_collision(solution_deg):
            print('COLLISION')
            return None
        
        # Проверка лимитов для плоской части
        if not self.is_within_limits(solution_deg.tolist()):
            # Ограничения углов
            q1 = np.clip(q1, np.deg2rad(self.angles_range[1][0]), np.deg2rad(self.angles_range[1][1]))
            q2 = np.clip(q2, np.deg2rad(self.angles_range[2][0]), np.deg2rad(self.angles_range[2][1]))
            q3 = np.clip(q3, np.deg2rad(self.angles_range[3][0]), np.deg2rad(self.angles_range[3][1]))

        return np.array([q1, q2, q3])
    
    def collision_gradient(self, p1, p2, angles):
        """Вычисляет градиент для избежания коллизий между двумя точками"""
        delta = p1 - p2
        distance = np.linalg.norm(delta)
        if distance > self.collision_margin:
            return np.zeros_like(angles)
        
        # Направление "отталкивания"
        direction = delta / (distance + 1e-6)
        
        # Вычисление якобиана для точек
        J1 = self.point_jacobian(angles, p1)
        J2 = self.point_jacobian(angles, p2)
        
        # Суммарный градиент
        return (J1 - J2).T @ direction * (self.collision_margin - distance)

    def point_jacobian(self, angles, point):
        """Аналитический якобиан позиции точки по углам"""
        q1, q2, q3 = angles
        L1, L2, L3 = self.L
        
        J = np.zeros(3)
        J[0] = -L1*np.sin(q1) - L2*np.sin(q1+q2) - L3*np.sin(q1+q2+q3)
        J[1] = -L2*np.sin(q1+q2) - L3*np.sin(q1+q2+q3)
        J[2] = -L3*np.sin(q1+q2+q3)
        
        return J
    
    def validate_solution(self, angles, target):
        if angles is None:
            return False
        return (
            self.is_within_limits(angles) and
            not self.check_collisions(angles) and
            self.position_error(angles, target) < 1e-3
        )
    
    def solve(self, target, max_time=0.05):
        """Основной метод решения"""
        # Инициализация предыдущими углами
        if self.prev_angles is None:
            self.prev_angles = np.zeros(4)
        
        # Шаг 1: Расчет базового поворота
        q0_new = self.analytical_base_rotation(target)
        q0_new = np.clip(q0_new, np.deg2rad(self.angles_range[0][0]), np.deg2rad(self.angles_range[0][1]))
        
        # Шаг 2: Проекция цели в рабочую плоскость
        r = sqrt(target[0]**2 + target[1]**2)
        target_plane = np.array([r, target[2]])
        
        # Шаг 3: Оптимизация плоской части с предыдущими углами
        planar_angles = self.solve_planar_ik(
            target_plane,
            self.prev_angles[1:]
        )

        # Динамическое изменение параметров
        safety_factor = 1.0
        for attempt in range(3):
            solution = self._solve_internal(target, safety_factor)
            if self.validate_solution(solution, target):
                return solution
            safety_factor *= 1.5  # Увеличиваем безопасную зону
            
        return None

        
        if planar_angles is not None:
            # Сборка полного решения
            solution = np.array([q0_new, planar_angles[0], planar_angles[1], planar_angles[2]])
            self.prev_angles = solution
        else:
            pass # self.prev_angles не меняются
        solution = None
            
        
        # # Проверка достижимости
        # final_pos = self.forward_kinematics(solution)
        # if np.linalg.norm(final_pos - target) > 1e-3:
        #     return None  # Решение не найдено
        
        
        return solution

    
    

if __name__ == "__main__":

    with open('config_arm.json', 'r') as file:
        config = json.load(file)

    ang_range = [
        [a - off, b - off] 
        for (a, b), off in zip(config['ang_range'], config['offset'])
    ]
    link_lengths = [l / 1000 for l in config['length']]

    iksolver = InverseKinematicsSolver(link_lengths, ang_range)

    target = np.array([0.1, 0.1, 0.1])

    angles = iksolver.solve(target)
    if angles is not None:
        print("Решение:", angles)
    else:
        print('pizdecnahuiblyat`')