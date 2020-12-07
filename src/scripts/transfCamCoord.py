#!/usr/bin/env python3
import numpy as np

"""
This function takes input from the camera depth [distance, angle], transforms it,
and produces output of the coordinates [x, y] for the next functions find & approach.
"""

# Input distance [mm] and angle [rad] from depth camera reading
# Output vertical vector with coordenates x and y [mm]


def transfCamCoord(dist, angle):
    return np.array([dist * np.cos(angle), dist * np.sin(angle)])

"""
# Example of transformation
d = 50
a = np.pi/6 # 30 degress
v = transfCamCoord(d, a)
print("x", v[0])
print("y", v[1])
"""
