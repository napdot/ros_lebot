#!/usr/bin/env python3
import numpy as np

def calc_angle(x_coord, adj):
    angle = np.arcsin((x_coord / adj))
    return angle
