from math import sqrt, atan2, cos

wheelAngle = [0, 120, 240]
wheelDistanceFromCenter = 20
robotAngularVelocity = 10
wheelSpeedToMainboardUnits = 10


def omni(speed_x, speed_y):
    try:
        robotDirectionAngle = atan2(speed_y, speed_x)
    except:
        robotDirectionAngle = 0.01

    robotSpeed = sqrt(speed_x * speed_x + speed_y * speed_y)

    wheelLinearVelocity = [0, 0, 0]
    wheelAngularSpeedMainboardUnits = [0, 0, 0]
    for i in range(3):
        wheelLinearVelocity[i] = robotSpeed * cos(robotDirectionAngle - wheelAngle[i]) \
                                      + wheelDistanceFromCenter * robotAngularVelocity

        wheelAngularSpeedMainboardUnits[i] = wheelLinearVelocity[i] * wheelSpeedToMainboardUnits

    return wheelLinearVelocity, wheelAngularSpeedMainboardUnits
