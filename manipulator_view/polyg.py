import numpy as np
from scipy.spatial import Delaunay
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def alpha_shape(points, alpha):
    """
    Построение Alpha Shape для облака точек.
    Возвращает список треугольников, представляющих внешнюю поверхность.
    """
    tetra = Delaunay(points)
    triangles = []
    for simplex in tetra.simplices:
        # Для каждого тетраэдра проверяем его грани
        for i in range(4):
            face = np.sort(np.delete(simplex, i))  # Удаляем одну вершину -> получаем грань
            circum_radius = np.linalg.norm(points[face[0]] - points[face[1]])  # Радиус окружности
            if circum_radius < alpha:  # Условие включения в оболочку
                triangles.append(face)
    return np.unique(triangles, axis=0)

if __name__ == '__main__':
    # Генерация облака точек с внутренней полостью
    np.random.seed(42)
    outer_points = np.random.uniform(-30, 30, size=(100, 3))
    inner_hole = np.random.uniform(-10, 10, size=(20, 3))
    points = np.vstack((outer_points, inner_hole))

    # Построение alpha-оболочки
    alpha = 10.0  # Параметр альфа: чем меньше, тем ближе оболочка к точкам
    triangles = alpha_shape(points, alpha)

    # Визуализация
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection="3d")

    # Добавляем точки
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, alpha=0.1, label="Точки")

    # Добавляем поверхность
    for tri in triangles:
        poly = Poly3DCollection([points[tri]], alpha=0.1, edgecolor='k')
        ax.add_collection3d(poly)

    # Настройка осей
    ax.set_xlim([-40, 40])
    ax.set_ylim([-40, 40])
    ax.set_zlim([-40, 40])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    plt.show()
