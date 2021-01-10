#!/usr/bin/env python3
import numpy as np

"""
This function takes input from the camera depth whether it locates the basket or not [True/False],
and produces output of the motors speed [M0, M1, M2] for the rotation around the ball at ballMinRange
it performs in case of not finding it.
When the camera finds the basket, the robot can start approaching the ball to throw it.
< Try to finish this stage with y = 0, x = ballMinRange and alligned Robot-Ball-Basket >
"""

aKI = np.array([[-1/np.sqrt(3), -1/3, 1/3], [1/np.sqrt(3), -1/3, 1/3], [0, 2/3, 1/3]]) # Inverse Matrix of Kinetic model
maxSpeedEnc = 100 # Serial wheels speed [-190, 190] with PID, [-255, 255] without PID
speedCut = 0.35 # [%] Percentage of motors "brake". 100% means full speed without reduction from the logic, not recommended nor useful

def findBasket(rot):
    """
    if not(found):
        # TODO
        # create logic to choose direction of rotation 1 (counterclockwise) or -1 (clockwise) depending on robot position in the court
        rotation = 1 # counterclockwise rotation by default
        m = np.dot(aKI, np.array([[1], [0], [rotation]]))
        mSer = np.rint(np.multiply(np.multiply(np.divide(m, np.max(np.absolute(m))), maxSpeedEnc), speedCut))
    else:
        mSer = np.array([[0], [0], [0]]) # stop when the basket is found
    return mSer
    """
    rotation = rot
    m = np.dot(aKI, np.array([[0], [1.3], [rotation]]))
    mSer = np.rint(np.multiply(np.multiply(np.divide(m, np.max(np.absolute(m))), maxSpeedEnc), speedCut))
    return mSer

"""
# Example of basket search
found = False
mSer = findBasket(found)
print("mSer", mSer)
print("found", found)
i = 0
print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> i = ", i)
while not(found):
    if i == 12:
        found = True
    mSer = findBasket(found)
    print("mSer", mSer)
    print("found", found)
    i = i + 1
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> i = ", i)
"""
