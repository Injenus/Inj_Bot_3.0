import numpy as np
import random
import json
import time

import dkp
from newton_raphson import find_closest_index, is_reachable, is_inside_polygon, is_segment_intersects_polygon, is_collision, find_closest_index
from main_ikp import create_plot_hist

random.seed(42)

counter_reach = 0
path_none = 0
thrid_none = 0
thresh = 15
def rrt_planner(q_start, pos_goal, angle_bounds, lengthes, max_iter=3000, step_size=20, k_step=3):
    global counter_reach, path_none, thrid_none, thresh
    if not is_reachable(pos_goal[0], pos_goal[1], lengthes):
        counter_reach += 1
        return None
    
    tree = [q_start]
    tree_parents = {tuple(q_start): None}
    
    for _ in range(max_iter):
        # Генерация случайной точки в пространстве конфигураций
        q_rand = np.array([
            random.uniform(b[0], b[1]) for b in angle_bounds
        ])
        
        # Поиск ближайшей вершины
        q_near = min(tree, key=lambda q: np.linalg.norm(q - q_rand))
        
        # Расширение в направлении q_rand
        direction = q_rand - q_near
        if np.linalg.norm(direction) < 1e-6:
            continue
        q_new = q_near + step_size * (direction / np.linalg.norm(direction))
        q_new = np.clip(q_new, [b[0] for b in angle_bounds], [b[1] for b in angle_bounds])
        
        # Проверка коллизий (зависит от окружения)
        if not is_collision(q_near, lengthes) or True:
            tree.append(q_new)
            tree_parents[tuple(q_new)] = tuple(q_near)

            # Проверка достижения цели
            pos_new = np.array(dkp.dkp_2d(lengthes, q_new))
            dist = np.linalg.norm(pos_new - pos_goal)
            step_size = max(dist/k_step, 1)
            
            if dist < thresh:
                path = reconstruct_path(tree_parents, q_new)
                if path[-1] is None:
                    path_none += 1
                return path[-1]
                return reconstruct_path(tree_parents, q_new)

    thrid_none += 1
    return None  # Путь не найден

def reconstruct_path(parents, q_end):
    path = [q_end]
    while parents[tuple(path[-1])] is not None:
        path.append(parents[tuple(path[-1])])
    return path[::-1]

if __name__ == "__main__":
    with open('config_arm.json') as file:
        config = json.load(file)

    # loaded_data = np.load('valid_area_data.npz')
    # # Извлекаем основной массив в список
    # main_loaded = loaded_data['main'].tolist()
    # # Восстанавливаем список массивов, сохраняя порядок
    # arrays_loaded = [loaded_data[f'array_{i}'] for i in range(len(main_loaded))]

    lengths = config['length']  # Длины звеньев в мм
    angle_bounds = np.array(config['ang_range'][1:]) - np.array(config['offset'][1:]).reshape(-1, 1) # Границы углов
    
    current_angles = np.array([0.0, 0.0, 0.0])  # Все звенья вытянуты
    current_pos = dkp.dkp_2d(lengths, current_angles)


    valid_pos = []
    max_r = sum(lengths)
    for r in range(-max_r, max_r, 4):
        for z in range(-max_r, max_r, 4):
            if (r**2+z**2)**0.5 < max_r:
                valid_pos.append(np.array([r, z]))

    random.shuffle(valid_pos)

    times = []
    
    thresh = 5.
    prev_pos = np.array([0., 327.])
    prev_q = np.array([0., 0., 0.])

    success = 0
    fail = 0
    none = 0

    total_iter = len(valid_pos)
    
    for i, target_pos in enumerate(valid_pos):
        if i%400:
            continue
        
        if not i%1000:
            print(f"{round(100*i/total_iter, 1)}%")
        
        timer = time.time()
        curr_q = rrt_planner(prev_q, target_pos, angle_bounds, lengths)
        times.append(round(time.time()-timer,3))

        if curr_q is not None:
            curr_pos = dkp.dkp_2d(lengths, curr_q)
            if np.linalg.norm(target_pos - curr_pos) < thresh:
                success += 1
            else:
                fail += 1

            prev_pos = curr_pos
            prev_q = curr_q
    
    create_plot_hist(times, 'rrt')

    total = len(valid_pos)
    small_suc = success

    with open('rrt_log.txt', 'w') as file:
        nl = '\n'
        file.write(f"Total: {total}\
                Suc: {success}, {success/total}{nl}\
                Fail: {fail}, {fail/total}{nl}\
                Reach nones: {counter_reach}, {counter_reach/total}{nl}\
                Path_none: {path_none}, {path_none/total}{nl}\
                Thrid none: {thrid_none}, {thrid_none/total}")

