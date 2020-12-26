#!/usr/bin/env python3
import numpy as np


def calc_angle(x_coord, adj):
    angle = 2 * np.arctan((x_coord/(2 * adj)))
    return angle
"""
x, ad = 200, 450
print(calc_angle(x, ad))
"""