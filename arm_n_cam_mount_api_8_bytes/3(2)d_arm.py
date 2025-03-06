import numpy as np
import json
from matplotlib.path import Path
import bisect


class ArmIKP():
    def __init__(self, lengths_main, ang_range, specific_valid_area, specific_theta_angles, global_valid_area, mode=2, init_q=np.array([0,0,0,0])):
        """
        Манипулятор из 4 звеньев (1 - основание).
        Первое звено вращательное в полокости основания,
        остальные - вращательные в перменликулярной основанию плоскости. 
        При mode=0 изменений в консутркции нет (равно как и нет никакого блять решения ОЗК),
        при mode=1 манипулятор рассматривается как 3 звенный - послднее звено считаеться неподвижным относительно второго,
        а если быть точным, его ориентация опрееделяет саи драйвер манипулятора, ориентируя в положение всегда параллеьное поверхности Земли.

        Т.к. ОЗК для 4-звенного манипулятора не решена, используется такой 3-звенный манипулятор.
        Целевые координаты предполгают перемещие конца 4-го звена в себя.
        Так как известно что 4 звено всегда параллеьно Поверхности, то в полярных координатах
        обе точки 4 звена имеют одинаковый угол и высоту, но разные расстояния, отличающиеся на длину звена.
        Поэтомy можно рассчитать целевые координат для конца 3-го звена, чтобы правильно спозиционировать манипулятор.
        Считаем целевые координаты (и любые другие конечные, если не оговррено иное) именно положением схавата. Чтобы вычислить положение конца второго звена,
        переводим целевые в цилиндрические, уменьшаем радиус, переводим в декартовы - считаем углы по ним.

        При mode=2 манипулятор рассмтаривается исключительно как 3-звенный - послднее звено игнорируется и управляется драйвером.
        Звено схвата так же не участвует в проверки коллизий и прочих условиях, подразумевается что оно никогда не выйдет за пределы рабочей области.

        """
        if mode not in (1, 2):
            raise ValueError('Only mode = 1 or 2 is supported')
        self.mode = mode
        self.lengths = np.array(lengths_main)
        self.ang_range = np.array(ang_range) # q0 (theta), q1, q2, q3;   q3 by IMU
        self.valid_areas = specific_valid_area
        self.specific_theta_angles = specific_theta_angles
        self.global_valid_area = global_valid_area

        self.q = init_q
        if mode == 1:
            self.xyz = self.dkp_2dof_add()
        elif mode == 2:
            self.xyz = self.dkp_2dof()
        self.cyclin = ArmIKP.cartesian_to_cylindrical(self.xyz, self.ang_range[0])

        self.target_xyz = None

    def is_reachable(self, x_, y_, plane_id=-1):
        l1,l2,l3 = self.lengths
        if self.mode == 1:
            is_r_valid = True
        elif self.mode == 2:
            lmax = max(l1, l2)
            lmin = min(l1,l2)
            is_r_valid = (lmax-lmin <= np.linalg.norm([x_,y_]) <= lmax+lmin)
        is_inside_polyg = self.is_inside_polygon(x_,y_,plane_id)
        return is_r_valid and not is_inside_polyg
    
    def is_inside_polygon(self, x, y, plane_id):
        path = self.global_valid_area if plane_id == -1 else self.valid_areas[plane_id]
        return path.contains_point((x,y), radius=-1e-9)
    

    @staticmethod
    def find_nearest_item_id(arr, item):
        # Используем bisect_left для нахождения позиции, куда можно вставить item
        pos = bisect.bisect_left(arr, item)
        
        # Если item меньше всех элементов в массиве, возвращаем индекс первого элемента
        if pos == 0:
            return 0
        # Если item больше всех элементов в массиве, возвращаем индекс последнего элемента
        if pos == len(arr):
            return len(arr) - 1
        
        # Сравниваем элемент на позиции pos и pos-1, чтобы найти ближайший
        before = arr[pos - 1]
        after = arr[pos]
        if after - item < item - before:
            return pos
        else:
            return pos - 1

    @staticmethod
    def limit_angle(theta_rad, range_rad):
        # Ограничение угла
        if range_rad[0] <= theta_rad <= range_rad[1]:
            return theta_rad
        else:
            # Зеркалирование угла
            if theta_rad < range_rad[0]:
                mirrored_angle = theta_rad + np.pi
            else:  # theta_deg > range[1]
                mirrored_angle = theta_rad - np.pi

            # Нормализация зеркального угла в пределах [-180, 180]
            mirrored_angle = (mirrored_angle + np.pi) % 2*np.pi - np.pi

            # Проверка, входит ли зеркальный угол в диапазон
            if range_rad[0] <= mirrored_angle <= range_rad[1]:
                return np.round(mirrored_angle, decimals=2)
            else:
                # Если снова выходит за диапазон, повторить зеркалирование
                return ArmIKP.limit_angle(mirrored_angle, range_rad)
    
    @staticmethod
    def cartesian_to_cylindrical(xyz, theta_range):
        x,y,z = xyz
        r = np.round(np.linalg.norm([x,y]))
        theta = np.arctan2(x,y)
        #theta = ArmIKP.limit_angle(theta, theta_range)
        h = np.round(z)
        return np.array([theta, r, h])

    @staticmethod
    def cylindrical_to_cartesian(cyclin):
        theta,r,h = cyclin
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        z = h
        return np.round(np.array([x,y,z]))

    @staticmethod
    def cartesian2d_to_polar(x,y, theta_range):
        r = np.round(np.linalg.norm([x, y]))
        theta = ArmIKP.limit_angle(np.arctan2(x,y), theta_range)
        return np.array([theta, r])

    
    def dkp_2dof_add(self, q=None):
        if q is None:
            q = self.q

        l1, l2, l3 = self.lengths
        q0, q1, q2, _ = q

        if self.mode == 1:
            r = l1*np.sin(q1) + l2*np.sin(q1+q2) + l3*np.sign(np.sin(q1+q2))
        else:
            # r = l1*np.sin(q1) + l2*np.sin(q1+q2) + l3*np.sin(q1+q2+_)
            pass
        
        x, y = r * np.sin(q0), r * np.cos(q0)
        z = l1*np.cos(q1) + l2*np.cos(q1+q2)

        return np.round(np.array([x,y,z]))
    
    def dkp_2dof(self, q=None):
        if q is None:
            q = self.q
        l1,l2,_ = self.lengths
        q0,q1,q2,_ = q

        if self.mode == 2:
            # nl='\n'
            # print(f'sin(q1)={np.sin(q1)}{nl}\
            #       sin(q1+q2)={np.sin(q1+q2)}{nl}\
            #       cos(q1)={np.cos(q1)}{nl}\
            #       cos(q1+q2)={np.cos(q1+q2)}{nl}')
            r = l1*np.sin(q1) + l2*np.sin(q1+q2)
            x,y = r*np.sin(q0), r*np.cos(q0)
            z = l1*np.cos(q1) + l2*np.cos(q1+q2)
        else:
            raise ValueError(f'Incorrect mode!')
        return np.round(np.array([x,y,z]))
    
    def ikp_2dof_add(self, target_xyz):
        l1,l2,l3 = self.lengths
        xt,yt,zt = target_xyz
        # if np.linalg.norm([xt,yt]) < l3:
        #     return [-7, -7, -7, -7] # mnr
            
        theta, r, h = ArmIKP.cartesian_to_cylindrical(target_xyz, self.ang_range[0])
        print(r, r-l3)
        xt_n, yt_n, zt_n = ArmIKP.cylindrical_to_cartesian((theta, r-l3, h)) # убираем звено схвата
        theta_n = ArmIKP.limit_angle(np.arctan2(xt_n,yt_n), self.ang_range[0])

        print(f'{theta}, {theta_n} | {target_xyz} {xt_n,yt_n,zt_n}')

        if abs(theta_n - theta) > 3*np.pi/180:
            raise ValueError(f"Too much difference btw theta!",'\n',f"theta={theta}, theta_n={theta_n}",'\n',f'{target_xyz} {xt_n,yt_n,zt_n}')
        
        if abs(theta_n) < 0.01:
            pseudo_x = 0
        else:
            pseudo_x = xt_n/np.sin(theta_n)
        pseudo_y = zt_n
        print(pseudo_x, pseudo_y)
        if self.is_reachable(pseudo_x, pseudo_y):
            q0 = theta_n
            q3 = 0
            rt_sq = np.sqrt(pseudo_x**2+pseudo_y**2)

            # Вычисление q₂ для конфигурации "локоть вверх"
            cos_theta2 = (rt_sq - l1**2 - l2**2) / (2 * l1 * l2)
            #cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
            q2 = -np.arccos(cos_theta2)  # Отрицательный угол (по часовой стрелке)
            
            # Вычисление q₁
            A = l1 + l2 * np.cos(q2)
            B = l2 * np.sin(q2)
            denominator = A**2 + B**2
            
            sin_theta1 = (A * pseudo_x - B * pseudo_y) / denominator
            cos_theta1 = (B * pseudo_x + A * pseudo_y) / denominator
            q1 = -np.arctan2(sin_theta1, cos_theta1)  # Отрицательный угол (по часовой стрелке)

            if abs(q1 - ArmIKP.limit_angle(q1, self.ang_range[1])) > 3*np.pi/180 or abs(q2 - ArmIKP.limit_angle(q2, self.ang_range[2])) > 3*np.pi/180:
                return [-8,-8,-8,-8] # imp

            q = np.round([q0,q1,q2,q3], decimals=2)
            q0_idx = ArmIKP.find_nearest_item_id(self.specific_theta_angles, q0)
            print('q', np.rad2deg(q))

            if self.is_reachable(np.linalg.norm([xt_n, yt_n]), zt_n, q0_idx):
                actual_xyz = self.dkp_2dof_add(q)
                if np.linalg.norm(np.array([actual_xyz]) - np.array([target_xyz])) < 1.0: # mm
                    return  q
                else:
                    raise ValueError('Решение слишком неправильное!', '\n', f'actual={actual_xyz}, target={target_xyz}')
            else:
                raise ValueError('Локоть оказался в запретной зоне!')
        else:
            return [-9, -9, -9, -9] # nr
        
    def ikp_2dof(self, target_xyz, position='up'):
        assert position in ['up', 'down']
        l1,l2,_ = self.lengths
        xt, yt, zt = target_xyz

        theta, r, h = ArmIKP.cartesian_to_cylindrical(target_xyz, self.ang_range[0])
        q0 = theta
        #print(f'q0 {np.rad2deg(q0)}')
        if abs(theta) < 0.01:
            pseudo_x = max(xt, yt)
        else:
            pseudo_x = xt/np.sin(theta)
        pseudo_y = zt
        #print('p',pseudo_x, pseudo_y)

        if self.is_reachable(pseudo_x, pseudo_y):
            pseudo_r = (pseudo_x**2+pseudo_y**2)**0.5
            #print('x y r', pseudo_x, pseudo_x, pseudo_r)

            q2 = np.pi - np.arccos((l1**2 + l2**2 - pseudo_r**2) / (2*l1*l2))
            alpha = np.arctan2(pseudo_x, pseudo_y)  #  Угол направления на цель (от оси Y по часовой стрелке); если ось Y вверх
            beta = np.arccos((l1**2 + pseudo_r**2 - l2**2) / (2*l1*pseudo_r) ) # Угол между l1 и линией к цели (beta)
            q1 = alpha - beta

            if position == 'down':
                q2 = -q2
                q1 = alpha + beta

            #print(np.rad2deg(q1), np.rad2deg(q2))

            # # q1_lim = ArmIKP.limit_angle(q1, self.ang_range[1])
            # # q2_lim = ArmIKP.limit_angle(q2, self.ang_range[2])
            # if abs(max(q1, q1_lim) - min(q1, q1_lim)) > 3*np.pi/180 or abs(max(q2, q2_lim) - min(q2, q2_lim)) > 3*np.pi/180:
            #     print(q1, q1_lim, '     ', q2, q2_lim)
            #     return [-8,-8,-8,-8] # imp
            
            q = np.round([q0,q1,q2,0], decimals=2)
            q0_idx = ArmIKP.find_nearest_item_id(self.specific_theta_angles, q0)
            #print('q', np.rad2deg(q))

            if self.is_reachable(np.linalg.norm([xt, yt]), zt, q0_idx):
                if position == 'down':
                    if not self.is_reachable(l1*np.sin(q1), l1*np.cos(q1), q0_idx): # локоить недоступен в низком положении
                        q1 = alpha - beta
                        q2 = -q2
                        if not self.is_reachable(l1*np.sin(q1), l1*np.cos(q1), q0_idx):
                            return [-6,-6,-6,-6] # локоить недоступен даже в высоком положении
                        
                actual_xyz = self.dkp_2dof(q)
                
                if np.linalg.norm(np.array([actual_xyz]) - np.array([target_xyz])) < 4.2: # mm
                    #print(np.round(actual_xyz))
                    return  q
                else:
                    return [-10,-10,-10,-10] # решение слишком сильно отличается от целевого
                    #raise ValueError('Решение слишком неправильное!', '\n', f'actual={actual_xyz}, target={target_xyz}')
            else:
                return [-7, -7, -7, -7]
                #raise ValueError('Целевая точка в запретной зоне!','wwdw')
        else:
            return [-9, -9, -9, -9] # nr
            




if __name__ == "__main__":
    with open('config_arm.json', 'r') as file:
        config = json.load(file)
    actual_ang_range = np.deg2rad([[a - off, b - off] for (a, b), off in zip(config['ang_range'], config['offset'])])

    valid_area_data = np.load('valid_area_data.npz')
    specific_theta_angles = valid_area_data['main']
    specific_valid_area = [Path(valid_area_data[f'array_{i}']) for i in range(len(specific_theta_angles.tolist()))]
    global_valid_area = Path(np.array([[-np.inf, -np.inf], [-np.inf, -170], [-210, -170], [-210, 0], [0, 0], [210, 0], [210, -170], [np.inf, -170], [np.inf, -np.inf]]))

    arm_inj_bot = ArmIKP(config['length'], actual_ang_range, specific_valid_area, specific_theta_angles, global_valid_area)

    print('res ', np.rad2deg(arm_inj_bot.ikp_2dof([-50, -50, 120], 'up')))
    #print(arm_inj_bot.dkp_2dof([np.deg2rad(0),np.deg2rad(-30),np.deg2rad(100),0]))