import numpy as np

class SCARAManipulator:
    def __init__(self):
        # Задаем параметры манипулятора (все размеры в мм)
        self.l1 = 100  # Длина первого звена (вращательное, без длины)
        self.l2 = 100  # Длина второго звена
        self.l3 = 50   # Длина третьего звена
        self.thickness = 10  # Толщина звеньев
        
        # Диапазоны углов (в градусах, переведены в положительную область)
        self.theta1_range = (0, 180)
        self.theta2_range = (0, 180)
        self.theta3_range = (0, 180)
        self.theta4_range = (0, 360)
        
        # Начальное положение (среднее между min и max)
        self.theta1 = np.mean(self.theta1_range)
        self.theta2 = np.mean(self.theta2_range)
        self.theta3 = np.mean(self.theta3_range)
        self.theta4 = np.mean(self.theta4_range)
    
    def forward_kinematics(self, theta1, theta2, theta3, theta4):
        """Вычисляет положение схвата на основе углов звеньев."""
        t1, t2, t3 = np.radians([theta1, theta2, theta3])
        
        x = self.l2 * np.cos(t1) + self.l3 * np.cos(t1 + t2)
        y = self.l2 * np.sin(t1) + self.l3 * np.sin(t1 + t2)
        z = theta4  # Ориентация схвата
        
        return np.array([x, y, z])
    
    def check_collision(self, theta1, theta2, theta3):
        """Проверяет коллизии манипулятора."""
        t1, t2 = np.radians([theta1, theta2])
        joint1_x = self.l2 * np.cos(t1)
        joint1_y = self.l2 * np.sin(t1)
        joint2_x = joint1_x + self.l3 * np.cos(t1 + t2)
        joint2_y = joint1_y + self.l3 * np.sin(t1 + t2)
        
        # Проверка самопересечений
        if np.hypot(joint1_x - joint2_x, joint1_y - joint2_y) < self.thickness:
            print("Коллизия: звенья пересекаются!")
            return True
        
        return False
    
    def inverse_kinematics(self, x_target, y_target, theta4):
        """Вычисляет углы звеньев для достижения указанного положения схвата."""
        d = np.hypot(x_target, y_target)
        
        if d > (self.l2 + self.l3):
            print("Цель недостижима!")
            return None
        
        cos_q2 = (x_target**2 + y_target**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        if abs(cos_q2) > 1:
            print("Цель недостижима!")
            return None
        
        q2 = np.arccos(cos_q2)
        q1 = np.arctan2(y_target, x_target) - np.arctan2(self.l3 * np.sin(q2), self.l2 + self.l3 * np.cos(q2))
        q3 = - (q1 + q2)
        
        theta1, theta2, theta3 = np.degrees([q1, q2, q3])
        
        # Сдвиг в положительную область
        theta1 += 90
        theta2 += 90
        theta3 += 90
        
        # Проверка ограничений
        if not (self.theta1_range[0] <= theta1 <= self.theta1_range[1]):
            print("Угол θ1 выходит за пределы!")
            return None
        if not (self.theta2_range[0] <= theta2 <= self.theta2_range[1]):
            print("Угол θ2 выходит за пределы!")
            return None
        if not (self.theta3_range[0] <= theta3 <= self.theta3_range[1]):
            print("Угол θ3 выходит за пределы!")
            return None
        
        if self.check_collision(theta1, theta2, theta3):
            print("Коллизия обнаружена! Движение невозможно.")
            return None
        
        return [theta1, theta2, theta3, theta4]
    
    def move_to(self, x, y, theta4):
        """Перемещает манипулятор в заданное положение."""
        angles = self.inverse_kinematics(x, y, theta4)
        if angles:
            print(f"Управляющие углы: {angles}")
        else:
            print("Перемещение невозможно.")

# Пример использования
if __name__ == "__main__":
    manipulator = SCARAManipulator()
    manipulator.move_to(50, 50, 0)
