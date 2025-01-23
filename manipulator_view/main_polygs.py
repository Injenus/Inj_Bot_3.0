import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

main_resolution = 24

class Manipulator3D:
    def __init__(self, base_length, link1_length, link2_length, link3_length):
        """
        Инициализация параметров манипулятора.
        :param base_length: Расстояние от опоры до оси вращения первого звена
        :param link1_length: Длина между осями первого и второго звена
        :param link2_length: Длина между осями второго и третьего звена
        :param link3_length: Длина от оси третьего звена до конца манипулятора
        """
        self.base_length = base_length
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
        
        return points

    def plot_workspace_polyg(self, resolution=main_resolution, filename=None):
        """
        Построение внешнего и внутреннего облаков точек рабочей области манипулятора
        с построением соответствующих полигонов.
        :param resolution: Разрешение углов (количество точек на угол)
        :param filename: Имя базового файла для сохранения графиков (без расширения)
        """
        points = self.compute_workspace(resolution)

        # Расчет центра и дистанций от точки до центра
        center = np.mean(points, axis=0)
        distances = np.linalg.norm(points - center, axis=1)

        # Выбор точек для внешнего и внутреннего облаков
        outer_points = points[distances > np.percentile(distances,80)]  # Внешние 25%
        inner_points = points[distances < np.percentile(distances, 0.1)]  # Внутренние 25%
        medium_points = points[(distances < np.percentile(distances, 67)) & (distances > np.percentile(distances, 33))]

        def plot_and_save(points_outer, points_inner, points_medium, facecolor_outer, facecolor_inner, facecolor_medium):
            """
            Вложенная функция для построения графиков и сохранения в файл.
            """
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            # Построение внешней оболочки
            outer_hull = ConvexHull(points_outer)
            for simplex in outer_hull.simplices:
                triangle = [points_outer[simplex[0]], points_outer[simplex[1]], points_outer[simplex[2]]]
                poly = Poly3DCollection([triangle], alpha=0.1, edgecolor='k', facecolor=facecolor_outer)
                ax.add_collection3d(poly)

            # Построение внутренней оболочки
            inner_hull = ConvexHull(points_inner)
            for simplex in inner_hull.simplices:
                triangle = [points_inner[simplex[0]], points_inner[simplex[1]], points_inner[simplex[2]]]
                poly = Poly3DCollection([triangle], alpha=0.1, edgecolor='k', facecolor=facecolor_inner)
                ax.add_collection3d(poly)

            medium_hull = ConvexHull(points_medium)
            for simplex in medium_hull.simplices:
                triangle = [points_medium[simplex[0]], points_medium[simplex[1]], points_medium[simplex[2]]]
                poly = Poly3DCollection([triangle], alpha=0.1, edgecolor='k', facecolor=facecolor_medium)
                #ax.add_collection3d(poly)

            # Добавление красной прозрачной плоскости на уровне z = 0
            xx, yy = np.meshgrid(np.linspace(-40, 40, 10), np.linspace(-40, 40, 10))
            zz = np.zeros_like(xx)
            ax.plot_surface(xx, yy, zz, color='red', alpha=0.3)

            # Установка одинакового масштаба по всем осям
            ax.set_xlim([-40, 40])
            ax.set_ylim([-40, 40])
            ax.set_zlim([-40, 40])

            ax.set_title("Рабочая область манипулятора")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")

            # Сохранение графика с суффиксом
            if filename:
                plt.savefig(filename, dpi=300)
            plt.show()

        # Построение внешней и внутренней оболочек
        plot_and_save(
            outer_points,
            inner_points,
            medium_points,
            facecolor_outer="blue",
            facecolor_inner="green",
            facecolor_medium="red"
        )

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
    filename = f"workspace_polygs_{i+1}.png"
    manipulator.plot_workspace_polyg(resolution=main_resolution, filename=filename)
