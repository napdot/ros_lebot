#!/usr/bin/env python3
import numpy as np

"""
This function takes input from the camera depth of the position of the ball [xBall, yBall] and the basket [xBasket, yBasket],
and produces output of the motors speed [M0, M1, M2] for the final approach & throw.
When the approach finishes, the robot is alligned with the ball and the basket, and with the thrower motor ready to grab & throw the ball.
< Try to finish this stage scoring the ball into the basket! >
"""

robotRadius = 132.5 # [mm] Radius of Robot structure (from center to omniwheels)
rangeOffset = 50 # [mm] Distance from Robot circumference where approach to ball stops
ballMinRange = robotRadius + rangeOffset # [mm] Distance to approach the ball before positioning for the throw
throwerContactDepth = 20 # [mm] Distance  within the robot from its circumference until the contact point ball-throwerMotor
grabRange = robotRadius - throwerContactDepth # [mm] Dinstance from the thrower motor to the ball
aKI = np.array([[np.sqrt(3)/3, 1/3, 1/3], [-np.sqrt(3)/3, 1/3, 1/3], [0, -2/3, 1/3]]) # Inverse Matrix of Kinetic model
maxSpeedEnc = 100 # Serial wheels speed [-190, 190] with PID, [-255, 255] without PID
speedCut = 0.2 # [%] Percentage of motors "brake". 100% means full speed without reduction from the logic, not recommended nor useful

def distanceToBall(xBall, yBall):
    return np.sqrt(np.square(xBall) + np.square(yBall))

# Input ball [xBall, yBall] and basket [xBasket, yBasket] position from camera depth
# Output motors speed mSer = [M0, M1, M2]
def approachThrow(xBall, yBall, xBasket, yBasket):
    """
    if distanceToBall(xBall, yBall) > grabRange:
        m = np.dot(aKI, np.array([[xBall], [yBall], [np.arctan2(yBasket - yBall, xBasket - xBall)]]))
        mSer = np.rint(np.multiply(np.multiply(np.divide(m, np.max(np.absolute(m))), maxSpeedEnc), speedCut))
    else:
        mSer = np.array([[0], [0], [0]]) # stop when approach finishes, robot is ready to throw the ball
    return mSer
    """
    m = np.dot(aKI, np.array([[xBall], [yBall], [np.arctan2(yBasket - yBall, xBasket - xBall)]]))
    mSer = np.rint(np.multiply(np.multiply(np.divide(m, np.max(np.absolute(m))), maxSpeedEnc), speedCut))
    return mSer


"""
# Example of throw approach
posXball = 150
posYball = 5
posXbasket = 620
posYbasket = 10
print("distanceToBall", distanceToBall(posXball, posYball))
mSer = approachThrow(posXball, posYball, posXbasket, posYbasket)
print("mSer", mSer)
i = 0
print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> i = ", i)
while distanceToBall(posXball, posYball) > grabRange:
    posXball -= posXball/10
    posYball -= posYball/10
    print("distanceToBall", distanceToBall(posXball, posYball))
    mSer = approachThrow(posXball, posYball, posXbasket, posYbasket)
    print("mSer", mSer)
    i = i + 1
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> i = ", i)
"""