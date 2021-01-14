
#!/usr/bin/env python3
import numpy as np

"""
This function takes input from the camera depth whether it locates a (or the closest) ball or not [True/False],
and produces output of the motors speed [M0, M1, M2] for the pure rotation it performs in case of not finding it.
When the camera finds the ball, the robot can start approaching to it.
< Try to finish this stage with y = 0 >
"""

maxSpeedEnc = 100 # Serial wheels speed [-190, 190] with PID, [-255, 255] without PID
speedCut = 0.3 # [%] Percentage of motors "brake". 100% means full speed without reduction from the logic, not recommended nor useful

def findBall(rot):
    """
    if not(found):
        # TODO
        # create logic to choose direction of rotation 1 (counterclockwise) or -1 (clockwise) depending on robot position in the court
        rotation = 1 # counterclockwise rotation by default
        mSer = np.rint(np.multiply(np.multiply(np.multiply(np.array([[1], [1], [1]]), rotation), maxSpeedEnc), speedCut))
    else:
        mSer = np.array([[0], [0], [0]]) # stop when the ball is found
    return mSer
    """
    rotation = rot
    mSer = np.rint(np.multiply(np.multiply(np.multiply(np.array([[1], [1], [1]]), rotation), maxSpeedEnc), speedCut))
    return mSer

"""
# Example of ball search
found = False
mSer = findBall(found)
print("mSer", mSer)
print("found", found)
i = 0
print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> i = ", i)
while not(found):
    if i == 12:
        found = True
    mSer = findBall(found)
    print("mSer", mSer)
    print("found", found)
    i = i + 1
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> i = ", i)
"""
