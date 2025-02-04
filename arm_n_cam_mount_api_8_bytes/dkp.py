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

    