#!/usr/bin/env python3

def calc_angle(x_coord):
    w = 640
    HFOV = 69.4
    angle = x_coord / (w / 2) * (HFOV / 2) * (3.1415/180)
    return angle

"""
print(calc_angle(-50))
"""
