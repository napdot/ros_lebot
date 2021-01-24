#!/usr/bin/env python3

import numpy as np

maxSpeedEnc = 100
speedCut = 0.03


def orient(loc):
    if loc < 0:
        rotation = -1
    elif loc > 0:
        rotation = 1
    mSer = np.rint(np.multiply(np.multiply(np.multiply(np.array([[1], [1], [1]]), rotation), maxSpeedEnc), speedCut))
    return mSer

