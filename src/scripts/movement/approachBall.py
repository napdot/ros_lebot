#!/usr/bin/env python3
import numpy as np

"""
This function takes input from the camera depth of the position of the ball [xBall, yBall],
and produces output of the motors speed [M0, M1, M2] for the approach.
When the approach finishes, the robot is facing the ball at ballMinRange distance of it.
< Try to finish this stage with y = 0 and x = ballMinRange >
"""

robotRadius = 132.5 # [mm] Radius of Robot structure (from center to omniwheels)
rangeOffset = 50 # [mm] Distance from Robot circumference where approach to ball stops
ballMinRange = robotRadius + rangeOffset # [mm] Distance to approach the ball before positioning for the throw
aKI = np.array([[np.sqrt(3)/3, 1/3, 1/3], [-np.sqrt(3)/3, 1/3, 1/3], [0, -2/3, 1/3]]) # Inverse Matrix of Kinetic model
maxSpeedEnc = 190 # Serial wheels speed [-190, 190] with PID, [-255, 255] without PID
speedCut = 0.2 # [%] Percentage of motors "brake". 100% means full speed without reduction from the logic, not recommended nor useful

def distanceToBall(xBall, yBall):
    return np.sqrt(np.square(xBall) + np.square(yBall))

# Input ball position [xBall, yBall] from camera depth
# Output motors speed mSer = [M0, M1, M2]
def approachBall(xBall, yBall):
    if distanceToBall(xBall, yBall) > ballMinRange:
        m = np.dot(aKI, np.array([xBall, yBall, np.arctan2(yBall, xBall)]))
        mSer = np.rint(np.multiply(np.multiply(np.divide(m, np.max(np.absolute(m))), maxSpeedEnc), speedCut))
    else:
        mSer = np.array([0, 0, 0])  # stop when approach finishes, robot is at ballMinRange distance of the ball

    return mSer

"""
# Example of ball approach
posXball = 560
posYball = 420
print("distanceToBall", distanceToBall(posXball, posYball))
mSer = approachBall(posXball, posYball)
print("mSer", mSer)
i = 0
print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> i = ", i)
while distanceToBall(posXball, posYball) > ballMinRange:
    posXball -= posXball/10
    posYball -= posYball/10
    print("distanceToBall", distanceToBall(posXball, posYball))
    mSer = approachBall(posXball, posYball)
    print("mSer", mSer)
    i = i + 1
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> i = ", i)
"""