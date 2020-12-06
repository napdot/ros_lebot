#!/usr/bin/env python3
def calc_angle(x_coord):
    cameraWidth = 640
    hfov = 69.4
    return (x_coord - (cameraWidth / 2) / (cameraWidth / 2)) * (hfov / 2)
