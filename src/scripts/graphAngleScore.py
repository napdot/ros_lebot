import numpy as np
from matplotlib import pyplot as plt


def score(min_dist, dist, angle):
    d = dist - min_dist
    if d == 0:
        return 0
    else:
        return (1 / np.sqrt(d)) * angle


vals = np.linspace(350, 5000, 60)

angles = []
p = 0.5
angle = 8

for val in vals:
    angles.append(score(350, val, 8))

plt.plot(vals, angles)
plt.show()
