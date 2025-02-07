import numpy as np
import math as m

def dkp_2d(lengths, angles):
    a, b, c = np.deg2rad(angles)
    l1, l2, l3 = lengths
    x = l1*np.sin(a) + l2*np.sin(a+b) + l3*np.sin(a+b+c)
    y = l1*np.cos(a) + l2*np.cos(a+b) + l3*np.cos(a+b+c)
    return x, y

def dkp_3d(lengths, angles):
    theta, a,b,c = np.deg2rad(angles)
    l1,l2,l3 = lengths

    z = 0 + l1*np.cos(a) + l2*np.cos(a+b) + l3*np.cos(a+b+c)
    r0 = 0
    r1 = r0 + l1*np.sin(a)
    r2 = r1 + l2*np.sin(a+b)
    r3 = r2 + l3*np.sin(a+b+c)
    x, y = r3 * np.sin(theta), r3 * np.cos(theta)

    return x,y,z

def arm_unit_coords_3d(lengths, angles):
    l1,l2,l3 = lengths
    q0,q1,q2,q3 = angles

    x0, y0, z0, r0 = 0, 0, 0, 0

    # Первая точка (после первого звена)
    r1 = r0 + l1*np.sin(q1)
    x1 = x0 + r1 * np.sin(q0)
    y1 = y0 + r1 * np.cos(q0)
    z1 = z0 + l1*np.cos(q1) 

    # Вторая точка (после второго звена)
    r2 = r1 + l2*np.sin(q1+q2)
    x2 = r2 * np.sin(q0)
    y2 = r2 * np.cos(q0)
    z2 = z1 + l2*np.cos(q1+q2)

    # Третья точка (после третьего звена — конец манипулятора)
    r3 = r2 + l3*np.sin(q1+q2+q3)
    x3 = r3 * np.sin(q0)
    y3 = r3 * np.cos(q0)
    z3 = z2 + l3*np.cos(q1+q2+q3)

    print(f'z3 {z3}')
    print(f'l1={np.linalg.norm(np.array([x1,y1,z1]) - np.array([x0,y0,z0]))}')
    print(f'l2={np.linalg.norm(np.array([x2,y2,z2]) - np.array([x1,y1,z1]))}')
    print(f'l3={np.linalg.norm(np.array([x3,y3,z3]) - np.array([x2,y2,z2]))}')

    # Координаты для отрисовки
    xs = [x0, x1, x2, x3]
    ys = [y0, y1, y2, y3]
    zs = [z0, z1, z2, z3]

    return xs, ys, zs