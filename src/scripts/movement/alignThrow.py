#!/usr/bin/env python3
import numpy as np

def align_throw(x0, y0, x1, y1):
    d = 450
    v0 = np.array([x0, y0])
    v1 = np.array([x1, y1])
    v = v0 - v1
    vlen = np.sqrt(np.power(v0, 2) + np.power(v1, 2))
    u = v / vlen
    return v0 + (d * u)

"""
v0 = [-2, 3]
v1 = [5, 5]
print(align(v0[0], v0[1], v1[0], v1[1]))
"""