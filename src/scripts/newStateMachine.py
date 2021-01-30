#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from objects import Ball, Robot
from lebot.msg import Depth_BallLocation
from lebot.msg import Depth_BasketLocation
from lebot.msg import Ref_Command

def ball_callback(data):
    x,y,d = data.x, data.y, data.d
    Ball.updateValues(Ball, x, y, d)
    return

def basket_callback(data):
    x,y,d = data.x, data.y, data.d
    Basket.updateValues(x, y, d)
    return

def referee_callback(data):
    global routine
    command_string = data.command
    if command_string == 'pause':
        routine = 'Pause'

    elif command_string == 'resume':
        routine = 'Initiate'


def do_Pause():
    return 'Pause'

def do_Initiate():
    return 'FindBall'

def do_FindBall():
    if Ball.isDetected:
        return 'OrientBall'
    else:
        """
        Rotation to find logic
        """
        return 'FindBall'

def do_OrientBall():
    if Ball.isDetected:
        if not Ball.isOrientedAngle:
            """
            Rotation to orient logic
            """
        else:
            return 'GetToBall'
    else:
        return 'FindBall'

def do_GetToBall():
    if Ball.isDetected:
        if not Ball.isOrientedAngle:
            return 'OrientToBall'
        else:
            if Ball.
                return 'FindBasket'
                return 'GetToBall'
    else:
        return 'FindBall'

def do_routine(cur_routine):
    if cur_routine == 'Pause':
        return do_Pause()
    elif cur_routine == 'Initiate':
        return do_Initiate()
    elif cur_routine == 'FindBall':
        return do_FindBall()
    elif cur_routine == 'OrientBall':
        return do_OrientBall()
    elif cur_routine == 'GetToBall':
        return do_GetToBall()


"""
How to implement.
theBall and theBasket object has the following properties:
    .isDetected         True if seen, False if unseen
    .x                  X position
    .y                  Y postion
    .d                  Distance
    .angle              Angle from camera midpoint
    .isOrientedPixel        
            


"""


if __name__ == '__main__':
    rospy.init_node('state_machine')
    Ball = theBall()
    Basket = theBasket()
    Robot = theRobot()
    ball_subscriber = rospy.Subscriber('/ball', Depth_BallLocation, ball_callback, queue_size=1)
    basket_subscriber = rospy.Subscriber('/basket', Depth_BasketLocation, basket_callback, queue_size=1)
    referee_subscriber = rospy.Subscriber('/referee', Ref_Command, referee_callback, queue_size=1)

    routine = 'Pause'
    while not rospy.is_shutdown:
        routine = do_routine(routine)
        rospy.spin()
