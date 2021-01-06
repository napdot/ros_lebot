#!/usr/bin/env python3
import numpy as np


def align_throw(x0, y0, x1, y1, d):
    v0 = np.array([x0, y0])
    v1 = np.array([x1, y1])
    v = v0 - v1
    vlen = np.sqrt(np.power(v1, 2) + np.power(v0, 2))
    u = v / vlen
    return v0 + (d * u)


"""
v0 = [15, 15]
v1 = [30, 36]
dist = 20
print(align_throw(v0[0], v0[1], v1[0], v1[1], dist))
"""

