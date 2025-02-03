import numpy as np
import math as m

def dkp_2d(lengths, angles):
    a, b, c = np.deg2rad(angles)
    l1, l2, l3 = lengths
    x = l1*np.sin(a) + l2*np.sin(a+b) + l3*np.sin(a+b+c)
    y = l1*np.cos(a) + l2*np.cos(a+b) + l3*np.cos(a+b+c)
    return x, y
    