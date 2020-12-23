import numpy as np


def omni_to_serial(x, y):
    force_mat = np.array([(np.sqrt(3)/3), 1/3, 1/3, (- np.sqrt(3)/3), 1/3, 1/3, 0, (-2/3), 1/3]).reshape(3, 3)
    w = np.arctan2(y, x)
    des_mat = np.array([x, y, w]).reshape(3, 1)
    ser = np.matmul(force_mat, des_mat)
    return int(ser[0]), int(ser[1]), int(ser[2])
