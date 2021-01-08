#!/usr/bin/env python3

def calc_angle(x_coord):
    w = 640
    HFOV = 74
    camera_mid_angle = x_coord / (w / 2) * (HFOV / 2) * (3.1415/180)
    angle = (3.1415 /2) - camera_mid_angle
    return angle

"""
print(calc_angle(-50))
"""
