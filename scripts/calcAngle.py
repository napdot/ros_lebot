def calc_angle(x_coord):
    cameraWidth = 1920
    hfov = 64
    return (x_coord - (cameraWidth / 2) / (cameraWidth / 2)) * (hfov / 2)
