import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class SCARAManipulator:
    def __init__(self):
        # Длины звеньев (мм)
        self.l1 = 0      # Первое звено - осевое вращательное, без длины
        self.l2 = 124    # Длина второго звена
        self.l3 = 63     # Длина третьего звена
        self.l4 = 149    # Длина четвертого звена (захват)
        
        # Диапазоны углов (в градусах)
        self.theta1_range = (15, 270)  # Вращение в плоскости XY
        self.theta2_range = (6, 261)   # Первый локоть
        self.theta3_range = (3, 258)   # Второй локоть
        self.theta4_range = (5, 260)   # Ориентация схвата

        # Начальное положение — средние углы
        self.theta1 = np.mean(self.theta1_range)
        self.theta2 = np.mean(self.theta2_range)
        self.theta3 = np.mean(self.theta3_range)
        self.theta4 = np.mean(self.theta4_range)

    def forward_kinematics(self, theta1, theta2, theta3, theta4):
        """Прямая кинематика — вычисляет координаты схвата по углам звеньев"""
        t1, t2, t3 = np.radians([theta1, theta2, theta3])

        # Координаты всех звеньев
        x1, y1, z1 = 0, 0, 0  # База
        x2 = self.l2 * np.cos(t2) * np.cos(t1)
        y2 = self.l2 * np.cos(t2) * np.sin(t1)
        z2 = self.l2 * np.sin(t2)

        x3 = x2 + self.l3 * np.cos(t2 + t3) * np.cos(t1)
        y3 = y2 + self.l3 * np.cos(t2 + t3) * np.sin(t1)
        z3 = z2 + self.l3 * np.sin(t2 + t3)

        x4 = x3
        y4 = y3
        z4 = z3 + self.l4  # Захват

        return np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]])

    def inverse_kinematics(self, x_target, y_target, z_target):
        """Обратная кинематика — вычисляет углы для заданной точки"""
        theta4 = 0  # Ориентация схвата

        # Угол первого звена
        theta1 = np.degrees(np.arctan2(y_target, x_target))

        # Перевод координат в систему отсчета второго звена
        d_xy = np.hypot(x_target, y_target)  # Расстояние в плоскости XY
        d = np.hypot(d_xy, z_target - self.l4)  # Полное расстояние

        # Проверка достижимости
        if d > (self.l2 + self.l3):
            print("Цель недостижима!")
            return None

        # Вычисление углов второго и третьего звена
        cos_q3 = (d**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        if abs(cos_q3) > 1:
            print("Цель недостижима!")
            return None

        theta3 = np.degrees(np.arccos(cos_q3))
        theta2 = np.degrees(np.arctan2(z_target - self.l4, d_xy)) - \
                 np.degrees(np.arctan2(self.l3 * np.sin(np.radians(theta3)), self.l2 + self.l3 * np.cos(np.radians(theta3))))

        # Проверка диапазонов углов
        if not (self.theta1_range[0] <= theta1 <= self.theta1_range[1]):
            print(f"Угол θ1 выходит за пределы! {theta1}")
            return None
        if not (self.theta2_range[0] <= theta2 <= self.theta2_range[1]):
            print(f"Угол θ2 выходит за пределы! {theta2}")
            return None
        if not (self.theta3_range[0] <= theta3 <= self.theta3_range[1]):
            print(f"Угол θ3 выходит за пределы! {theta3}")
            return None

        return [theta1, theta2, theta3, theta4]

    def plot_manipulator(self, theta1, theta2, theta3, theta4):
        """Визуализация манипулятора"""
        positions = self.forward_kinematics(theta1, theta2, theta3, theta4)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim([-200, 200])
        ax.set_ylim([-200, 200])
        ax.set_zlim([0, 300])

        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], marker='o', linestyle='-', color='b')
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.show()

    def move_to(self, x, y, z):
        """Перемещение в заданное положение"""
        angles = self.inverse_kinematics(x, y, z)
        if angles:
            print(f"Управляющие углы: {angles}")
            self.plot_manipulator(*angles)
        else:
            print("Перемещение невозможно.")

# Проверка начального положения
if __name__ == "__main__":
    manipulator = SCARAManipulator()
    
    # Берём координаты, соответствующие средним углам
    pos = manipulator.forward_kinematics(manipulator.theta1, manipulator.theta2, manipulator.theta3, manipulator.theta4)[-1]
    manipulator.move_to(*pos)
