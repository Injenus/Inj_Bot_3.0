def solution():
    pass

import numpy as np
from scipy.optimize import minimize
import json
import random
import time
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution
from scipy.optimize import *

random.seed(42)

def forward_kinematics(theta, lengths):
    """Прямая кинематика: вычисление позиции конца манипулятора."""
    theta1, theta2, theta3 = theta
    L1, L2, L3 = lengths
    
    r = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2) + L3 * np.sin(theta1 + theta2 + theta3)
    z = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
    
    return np.array([r, z])

def compute_jacobian(q, lengths):
    """Вычисление матрицы Якоби для текущих углов."""
    q1, q2, q3 = q
    L1, L2, L3 = lengths
    
    sum_theta12 = q1 + q2
    sum_theta123 = sum_theta12 + q3
    
    dx_dq1 = L1 * np.cos(q1) + L2 * np.cos(sum_theta12) + L3 * np.cos(sum_theta123)
    dx_dq2 = L2 * np.cos(sum_theta12) + L3 * np.cos(sum_theta123)
    dx_dq3 = L3 * np.cos(sum_theta123)
    
    dy_dtheta1 = -L1 * np.sin(q1) - L2 * np.sin(sum_theta12) - L3 * np.sin(sum_theta123)
    dy_dtheta2 = -L2 * np.sin(sum_theta12) - L3 * np.sin(sum_theta123)
    dy_dtheta3 = -L3 * np.sin(sum_theta123)
    
    J = np.array([
        [dx_dq1, dx_dq2, dx_dq3],
        [dy_dtheta1, dy_dtheta2, dy_dtheta3]
    ])
    return J

def clip_angles(angles, angle_bounds):
    """Ограничение углов заданными пределами."""
    # print(f'{angles}___{angle_bounds}')
    return np.clip(angles, [b[0] for b in angle_bounds], [b[1] for b in angle_bounds])

def small_delta_ik(current_angles, current_pos, target_pos, lengths, angle_bounds, max_step=0.1):
    """Решение для малых изменений через псевдообратный Якобиан."""
    delta_pose = np.array(target_pos) - np.array(current_pos)
    J = compute_jacobian(current_angles, lengths)
    J_pseudo = np.linalg.pinv(J)
    
    delta_theta = J_pseudo @ delta_pose
    
    # Ограничение максимального шага
    step_norm = np.linalg.norm(delta_theta)
    if step_norm > max_step:
        delta_theta *= max_step / step_norm
    
    new_angles = current_angles + delta_theta
    return clip_angles(new_angles, angle_bounds)

def large_delta_ik(initial_angles, target_pos, lengths, angle_bounds, max_r=327):
    """Решение через оптимизацию с использованием scipy."""
    def objective(theta):
        pos = forward_kinematics(theta, lengths)
        position_error = np.sum((pos - target_pos)**2)
        smoothness_penalty = 0.1 * np.sum((theta - initial_angles)**2)  # Плавность
        # if (np.linalg.norm(pos) > max_r):
        #     return 1e6
        return position_error + smoothness_penalty
    
    

    result = differential_evolution(
        objective,
        bounds=angle_bounds,
        maxiter=1000,
        popsize=15,
        tol=0.1,  # Допуск по точности 
        updating='deferred'
    )
    
    

    if not result.success:
        return None
    
    return clip_angles(result.x, angle_bounds)

def create_plot_hist(times, name):
    plt.figure(figsize=(10, 6))
    plt.hist(times, bins=int(np.sqrt(len(times))) if len(times) > 10 else 5)

    mean_val = np.mean(times)
    plt.axvline(mean_val, color='k', linestyle='dashed', linewidth=2, 
                label=f'Среднее: {mean_val:.2f} сек')
    
    plt.title(f"Распределение времени для {name}\n"
            f"Всего точек: {len(times)}")
    plt.xlabel('Время решения (секунды)')
    plt.ylabel('Количество точек')
    plt.legend()
    
    plt.savefig(f"{name}hist.png", 
                bbox_inches='tight', 
                dpi=150)
    plt.close()


if __name__ == "__main__":
    


    with open('config_arm.json', 'r') as file:
        config = json.load(file)

    lengths = config['length']  # Длины звеньев в мм
    angle_bounds = np.deg2rad(np.array(config['ang_range'][1:]) - np.array(config['offset'][1:]).reshape(-1, 1)).tolist()  # Границы углов
    
    # Начальное состояние
    current_angles = np.array([0.0, 0.0, 0.0])  # Все звенья вытянуты
    current_pos = forward_kinematics(current_angles, lengths)
    #current_pos = np.array([1., 1., 10.])
    print(current_pos)

    # Малые изменения
    target_small = current_pos + np.array([5, 5])  # Сдвиг на 5 мм
    new_angles_small = small_delta_ik(current_angles, current_pos, target_small, lengths, angle_bounds)
    
    # Большие изменения
    target_large = np.array([150, 50])  # Новая целевая позиция
    new_angles_large = large_delta_ik(current_angles, target_large, lengths, angle_bounds)
    
    print("Малые изменения углов:", np.degrees(new_angles_small))
    if new_angles_large is not None:
        print("Большие изменения углов:", np.degrees(new_angles_large))
    
    valid_pos = []
    max_r = sum(lengths)
    for r in range(-max_r, max_r, 4):
        for z in range(-max_r, max_r, 4):
            if (r**2+z**2)**0.5 < max_r:
                valid_pos.append(np.array([r, z]))

    # step = 101
    # for i in range(0, len(valid_pos), step):
    #     if not i%3:
    #         a, b, c = valid_pos[:(i-1)*step], valid_pos[(i-1)*step:i*step], valid_pos[i*step:]
    #         random.shuffle(b)
    #         valid_pos = a + b + c

    random.shuffle(valid_pos)

    times_small = []
    times_large = []
    
    small_thresh = 0.
    prev_pos = np.array([0., 327.])
    prev_q = np.array([0., 0., 0.])

    mask = (np.linalg.norm(valid_pos - prev_pos, axis=1) <= small_thresh)

    success_small_count = 0
    success_large_count = 0
    fail_small_count = 0
    fail_large_count = 0
    none_large_count = 0

    total_iter = len(valid_pos)
    
    for i, target_pos in enumerate(valid_pos[:-1]):
        
        if not i%1000:
            print(f"{round(100*i/total_iter, 1)}%")
        
        timer = time.time()
        is_mask = np.all(mask[i])

        if is_mask:
            curr_q = small_delta_ik(prev_q, prev_pos, target_pos, lengths, angle_bounds)
            times_small.append(round(time.time()-timer,3))
        else:
            curr_q = large_delta_ik(prev_q, target_pos, lengths, angle_bounds, max_r)
            times_large.append(round(time.time()-timer,3))
            if curr_q is None:
                none_large_count += 1
                continue

        curr_pos = forward_kinematics(curr_q, lengths)
        if np.linalg.norm(target_pos - curr_pos) <= 5:
            if is_mask:
                success_small_count += 1
            else:
                success_large_count += 1
        else:
            if is_mask:
                fail_small_count += 1
            else:
                fail_large_count += 1
        
        prev_pos = curr_pos
        prev_q = curr_q


    # small_success = round(success_small_count/fail_small_count,2)
    # large_success = round(success_large_count/(fail_large_count+none_large_count), 2)

    # total_success = round((success_large_count+success_small_count)/(fail_large_count+fail_small_count+none_large_count),2)

    # large_fail = round(fail_large_count/success_large_count,2)
    # nones_by_large_succ = round(none_large_count/success_large_count,2)
    # nones_by_large_fail = round(none_large_count/fail_large_count, 2)

    # with open('ikp_main_log.txt', 'a') as file:
    #     nl = '\n'
    #     file.write(f"Small succ: {small_success}{nl}\
    #                Large succ: {large_success}{nl}\
    #                Total succ: {total_success}{nl}\
    #                Large_fail: {large_fail}{nl}\
    #                Nones_by_large_succ: {nones_by_large_succ}{nl}\
    #                Nones_by_large_fail: {nones_by_large_fail}\
    #                Всего координат: {len(valid_pos)}")
        
    create_plot_hist(times_large, 'large')
    create_plot_hist(times_small, 'small')

    total = len(valid_pos)
    small_suc = success_small_count

    with open('ikp_main_log.txt', 'w') as file:
        nl = '\n'
        file.write(f"Total: {total}\
                Small suc: {success_small_count},{success_small_count/total}{nl}\
                Small fail: {fail_small_count},{fail_small_count/total}{nl}\
                Large succ: {success_large_count},{success_large_count/total}{nl}\
                Large_fail: {fail_large_count},{fail_large_count/total}{nl}\
                Nones: {none_large_count},{none_large_count/total}{nl}")

