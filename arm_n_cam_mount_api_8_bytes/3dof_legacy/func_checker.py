import numpy as np

from newton_raphson import jacobian, f
from newton_raphson import *
from dkp import *





def check_jacobian():
    lengths = [2, 3, 4]
    angles = np.radians([30, 45, 60])
    J_analytical = jacobian(lengths, angles)
    
    # Численный якобиан
    eps = 1e-6
    J_num = np.zeros((2, 3))
    for i in range(3):
        angles_plus = angles.copy()
        angles_plus[i] += eps
        F_plus = f(angles_plus, 0, 0, lengths)
        
        angles_minus = angles.copy()
        angles_minus[i] -= eps
        F_minus = f(angles_minus, 0, 0, lengths)
        
        J_num[:, i] = (F_plus - F_minus) / (2 * eps)
    
    print("Аналитический якобиан:\n", J_analytical)
    print("Численный якобиан:\n", J_num)

#check_jacobian()

# # Тест 1: Точка внутри области
# print(newton_raphson(
#     [0, 0, 0], 
#     x_target=3, y_target=0,
#     lengths=[1, 1, 1],
#     ang_range=[[None, None],[-90,90], [-90,90], [-90,90]]
# ))  # Ожидаем ~[90.0, 0.0, 0.0]


legths = [124, 63, 140]
x_target = -200
y_target = -100

ang = newton_raphson(
    [0, 0, 0],
    x_target=x_target, y_target=y_target,
    lengths=legths,
    ang_range=[[None, None],[-123,123], [-125,125], [-123,123]]
) # Ожидаем [0.0, 0.0, 0.0]
print(ang)
print(f'target is {x_target, y_target}   actual is = {dkp_2d(legths, ang)}')