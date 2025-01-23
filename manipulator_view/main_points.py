import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

main_resolution = 15

class Manipulator3D:
    def __init__(self, base_length, link1_length, link2_length, link3_length):
        """
        Инициализация параметров манипулятора.
        :param base_length: Расстояние от опоры до оси вращения первого звена
        :param link1_length: Длина между осями первого и второго звена
        :param link2_length: Длина между осями второго и третьего звена
        :param link3_length: Длина от оси третьего звена до конца манипулятора
        """
        self.base_length = 0
        self.link1_length = link1_length
        self.link2_length = link2_length
        self.link3_length = link3_length

    def compute_workspace(self, resolution=main_resolution):
        """
        Вычисление рабочей области манипулятора.
        :param resolution: Разрешение углов (количество точек на угол)
        :return: массив координат рабочей области (x, y, z)
        """
        # Углы поворота для всех звеньев
        theta0 = np.linspace(-np.pi * 3 / 4, np.pi * 3 / 4, resolution)  # Основание
        theta1 = np.linspace(-np.pi * 3 / 4, np.pi * 3 / 4, resolution)  # Первое звено
        theta2 = np.linspace(-np.pi * 3 / 4, np.pi * 3 / 4, resolution)  # Второе звено
        theta3 = np.linspace(-np.pi * 3 / 4, np.pi * 3 / 4, resolution)  # Третье звено

        # Создаем сетку всех комбинаций углов
        T0, T1, T2, T3 = np.meshgrid(theta0, theta1, theta2, theta3, indexing='ij')

        # Вычисляем координаты всех точек
        x = (self.link1_length * np.sin(T1) +
            self.link2_length * np.sin(T1 + T2) +
            self.link3_length * np.sin(T1 + T2 + T3)) * np.sin(T0)
        
        y = (self.link1_length * np.sin(T1) +
            self.link2_length * np.sin(T1 + T2) +
            self.link3_length * np.sin(T1 + T2 + T3)) * np.cos(T0)
        
        z = (self.base_length +
            self.link1_length * np.cos(T1) +
            self.link2_length * np.cos(T1 + T2) +
            self.link3_length * np.cos(T1 + T2 + T3))

        # Объединяем координаты в массив точек
        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)

        theta0 = np.arctan2(points[:, 1], points[:, 0])  # arctan2(y, x) возвращает угол в радианах
        theta0 = np.mod(theta0 + np.pi, 2 * np.pi) - np.pi
        mask = (theta0 >= -np.pi * 3 / 4) & (theta0 <= np.pi * 3 / 4)
        points = points[mask]

        print(f'min {np.min(np.linalg.norm(np.array((0,0,0)) - points, axis=1))}')

        return points

    def plot_workspace_points(self, resolution=main_resolution, filename=None):
        """
        Построение внешнего и внутреннего облаков точек рабочей области манипулятора.
        :param resolution: Разрешение углов (количество точек на угол)
        :param filename: Имя файла для сохранения графика (если указано)
        """
        points = self.compute_workspace(resolution)

        # Расчет центра и дистанций от точки до центра
        center = np.mean(points, axis=0)
        distances = np.linalg.norm(points - center, axis=1)

        # Выбор точек для внешнего и внутреннего облаков
        outer_points = points[distances > np.percentile(distances, 80)]  # Внешние 25%
        inner_points = points[distances < np.percentile(distances, 4)]  # Внутренние 25%
        medium_points = points[(distances < np.percentile(distances, 67)) & (distances > np.percentile(distances, 33))]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Построение облаков точек
        ax.scatter(outer_points[:, 0], outer_points[:, 1], outer_points[:, 2], c='blue', label='Outer Points', s=1)
        ax.scatter(inner_points[:, 0], inner_points[:, 1], inner_points[:, 2], c='green', label='Inner Points', s=1)
        #ax.scatter(medium_points[:, 0], medium_points[:, 1], medium_points[:, 2], c='red', label='Inner Points', s=1)

        # Добавление красной плоскости на нулевой высоте
        xx, yy = np.meshgrid(np.linspace(np.min(points[:, 0]), np.max(points[:, 0]), 10),
                             np.linspace(np.min(points[:, 1]), np.max(points[:, 1]), 10))
        zz = np.zeros_like(xx)
        ax.plot_surface(xx, yy, zz, color='red', alpha=0.3)

         # Установка одинакового масштаба по всем осям
        ax.set_xlim([-40, 40])
        ax.set_ylim([-40, 40])
        ax.set_zlim([-40, 40])

        ax.set_title("Рабочая область манипулятора (облака точек)")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend()

        if filename:
            plt.savefig(filename, dpi=300)
        plt.show()


# Создание конфигураций манипулятора и построение их рабочих областей
manipulators = [
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=6.3, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=8.5, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=10.5, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=12.5, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=14.5, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=16.5, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=18.5, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=20.5, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=22.5, link3_length=13.5),
    Manipulator3D(base_length=2.5, link1_length=12.5, link2_length=24.5, link3_length=13.5)
]

for i, manipulator in enumerate(manipulators):
    filename = f"workspace_points_{i+1}.png"
    manipulator.plot_workspace_points(resolution=main_resolution, filename=filename)
