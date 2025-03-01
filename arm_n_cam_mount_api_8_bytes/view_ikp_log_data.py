import os
import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main(directory):

    # Регулярные выражения для парсинга файлов
    jpg_pattern = re.compile(r"(?P<name>.+) x=(?P<x>-?\d+), y=(?P<y>-?\d+), z=(?P<z>-?\d+), time=(?P<time>\d+\.\d+)\.jpg")
    txt_unreachable_pattern = re.compile(r"Недостиж (?P<name>.+) x=(?P<x>-?\d+), y=(?P<y>-?\d+), z=(?P<z>-?\d+), time=(?P<time>\d+\.\d+)\.txt")
    txt_outzone_pattern = re.compile(r"Вне з (?P<name>.+) x=(?P<x>-?\d+), y=(?P<y>-?\d+), z=(?P<z>-?\d+), time=(?P<time>\d+\.\d+)\.txt")

    # Списки координат
    jpg_points = []
    unreachable_points = []
    outzone_points = []

    # Списки времен
    jpg_times = []
    unreachable_times = []
    outzone_times = []

    # Проходим по всем файлам в директории
    for filename in os.listdir(directory):
        jpg_match = jpg_pattern.match(filename)
        unreachable_match = txt_unreachable_pattern.match(filename)
        outzone_match = txt_outzone_pattern.match(filename)

        if jpg_match:
            x, y, z = map(int, [jpg_match.group("x"), jpg_match.group("y"), jpg_match.group("z")])
            sol_time = float(jpg_match.group("time"))
            jpg_points.append((x, y, z))
            jpg_times.append(sol_time)

        elif unreachable_match:
            x, y, z = map(int, [unreachable_match.group("x"), unreachable_match.group("y"), unreachable_match.group("z")])
            sol_time = float(unreachable_match.group("time"))
            unreachable_points.append((x, y, z))
            unreachable_times.append(sol_time)

        elif outzone_match:
            x, y, z = map(int, [outzone_match.group("x"), outzone_match.group("y"), outzone_match.group("z")])
            sol_time = float(outzone_match.group("time"))
            outzone_points.append((x, y, z))
            outzone_times.append(sol_time)

    # Функция для вычисления min, max и среднего времени
    def calc_time_stats(times):
        return round(min(times), 2) if times else None, round(max(times), 2) if times else None, round(np.mean(times), 2) if times else None

    # Вычисляем статистику времени
    jpg_time_stats = calc_time_stats(jpg_times)
    unreachable_time_stats = calc_time_stats(unreachable_times)
    outzone_time_stats = calc_time_stats(outzone_times)

    print(f'{directory}:')
    # Вывод статистики
    jpg_num = len(jpg_points)
    unreachable_num = len(unreachable_points)
    outzone_num = len(outzone_points)
    all_num = jpg_num + unreachable_num + outzone_num

    print(f"JPG файлы {round(100*jpg_num/all_num,2)}% (основные):", jpg_time_stats)
    print(f"Недостижимые точки {round(100*unreachable_num/all_num,2)}%:", unreachable_time_stats)
    print(f"Вне зоны {round(100*outzone_num/all_num,2)}%:", outzone_time_stats)
    print()

if __name__ == '__main__':
    main('ikp_log_Path_segments_100_random')
    main('ikp_log_Path_segments_100_prev')
    main('ikp_log_Path_segments_1000_random')
    main('ikp_log_Path_segments_1000_prev')
    main('ikp_log_Path_segments_42_real_prev_tol_1.0')



# Визуализация в 3D
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# # Функция для нанесения точек
# def plot_points(ax, points, color, size, label):
#     if points:
#         x, y, z = zip(*points)
#         ax.scatter(x, y, z, c=color, s=size, label=label)

# # Наносим точки
# plot_points(ax, jpg_points, "blue", 1, "Основные (jpg)")
# plot_points(ax, outzone_points, "red", 3, "Вне зоны (txt)")
# plot_points(ax, unreachable_points, "yellow", 3, "Недостижимые (txt)")

# # Настройки графика
# ax.set_xlabel("X координата")
# ax.set_ylabel("Y координата")
# ax.set_zlabel("Z координата")
# ax.set_title(directory)
# ax.legend()
# #plt.show()

def save_ply():
    output_file = f"{directory}_point_cloud.ply"

    # Объединяем все точки и их цвета
    all_points = []
    all_colors = []

    # Добавляем основные (синие) точки
    for x, y, z in jpg_points:
        all_points.append((x, y, z))
        all_colors.append((0, 0, 255))  # Синий

    # Добавляем "Вне зоны" (красные) точки
    for x, y, z in outzone_points:
        all_points.append((x, y, z))
        all_colors.append((255, 0, 0))  # Красный

    # Добавляем "Недостижимые" (желтые) точки
    for x, y, z in unreachable_points:
        all_points.append((x, y, z))
        all_colors.append((255, 255, 0))  # Желтый

    # Записываем в PLY-файл
    with open(output_file, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(all_points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("property uchar alpha\n")  # Добавляем альфа-канал
        f.write("end_header\n")

        for (x, y, z), (r, g, b) in zip(all_points, all_colors):
            f.write(f"{x} {y} {z} {r} {g} {b} 255\n")  # Добавили 255 в конец

    print(f"3D-модель сохранена в {output_file}")


def create_sphere(center, radius, segments=12, rings=12):
    """Создаёт вершины, грани и нормали для сферы."""
    vertices = []
    faces = []
    normals = []

    cx, cy, cz = center
    index_offset = len(vertices)

    for i in range(rings + 1):
        theta = np.pi * i / rings
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        for j in range(segments):
            phi = 2 * np.pi * j / segments
            x = cx + radius * np.cos(phi) * sin_theta
            y = cy + radius * np.sin(phi) * sin_theta
            z = cz + radius * cos_theta
            vertices.append((x, y, z))

            # Вычисляем нормаль
            nx = np.cos(phi) * sin_theta
            ny = np.sin(phi) * sin_theta
            nz = cos_theta
            normals.append((nx, ny, nz))

    for i in range(rings):
        for j in range(segments):
            next_j = (j + 1) % segments
            v1 = index_offset + i * segments + j
            v2 = index_offset + i * segments + next_j
            v3 = index_offset + (i + 1) * segments + j
            v4 = index_offset + (i + 1) * segments + next_j

            if i != 0:
                faces.append((v1, v2, v3))
            if i != rings - 1:
                faces.append((v3, v2, v4))

    return vertices, normals, faces

def save_obj(filename, jpg_points, outzone_points, unreachable_points, base_radius=1.0):
    obj_file = f"{filename}_spheres.obj"
    mtl_file = f"{filename}_spheres.mtl"

    vertices = []
    normals = []
    faces = []
    materials = {
        "blue": (0, 0, 1),
        "red": (1, 0, 0),
        "yellow": (1, 1, 0)
    }

    with open(mtl_file, "w") as mtl:
        for name, (r, g, b) in materials.items():
            mtl.write(f"newmtl {name}\n")
            mtl.write(f"Kd {r} {g} {b}\n")
            mtl.write("Ka 0.2 0.2 0.2\n")
            mtl.write("d 1.0\n\n")

    index_offset = 1

    with open(obj_file, "w") as obj:
        obj.write(f"mtllib {os.path.basename(mtl_file)}\n")

        for color, points, radius in [("blue", jpg_points, base_radius), 
                                      ("red", outzone_points, base_radius * 3), 
                                      ("yellow", unreachable_points, base_radius * 3)]:
            obj.write(f"usemtl {color}\n")

            for x, y, z in points:
                v, n, f = create_sphere((x, y, z), radius)
                v_offset = len(vertices)

                vertices.extend(v)
                normals.extend(n)
                faces.extend([(a + v_offset + index_offset, b + v_offset + index_offset, c + v_offset + index_offset) for a, b, c in f])

            index_offset += len(vertices)

        for x, y, z in vertices:
            obj.write(f"v {x} {y} {z}\n")

        for nx, ny, nz in normals:
            obj.write(f"vn {nx} {ny} {nz}\n")

        for a, b, c in faces:
            obj.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")

    print(f"3D-модель сохранена в {obj_file} и {mtl_file}")

#save_ply()
#save_obj(directory, jpg_points, outzone_points, unreachable_points, base_radius=1.0)