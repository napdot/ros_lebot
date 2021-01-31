#!/usr/bin/env python3

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
import numpy as np
from .objects import theBall, theRobot, theBasket, theLine
from std_msgs.msg import String
from lebot.msg import Depth_BallLocation
from lebot.msg import Depth_BasketLocation
from lebot.msg import Ref_Command
from lebot.msg import LineLocation


def ball_callback(data):
    x, y, d = data.x, data.y, data.d
    Ball.updateValues(Ball, x, y, d)
    return


def basket_callback(data):
    x, y, d = data.x, data.y, data.d
    Basket.updateValues(x, y, d)
    return


def referee_callback(data):
    global routine
    command_string = data.command
    if command_string == 'pause':
        routine = 'Pause'

    elif command_string == 'resume':
        routine = 'Initiate'


def line_callback(data):
    x1, y1, x2, y2 = data.x1, data.y1, data.x2, data.y2
    Line.updateValues(x1, y1, x2, y2)
    return


def do_Pause():
    return 'Pause'


def do_Initiate():
    return 'FindBall'


def do_FindBall():
    global stuck_counter
    if stuck_counter >= stuck_counter_max:
        stuck_counter = 0
        return 'Stuck'
    stuck_counter = stuck_counter + 1

    if Ball.isDetected:
        return 'OrientBall'
    else:   # Rotate and findBall.
        Robot.rotate(find_speed)
        return 'FindBall'


def do_OrientBall():
    global stuck_counter
    if stuck_counter >= stuck_counter_max:
        stuck_counter = 0
        return 'Stuck'
    stuck_counter = stuck_counter + 1

    if Ball.isDetected:
        if not Ball.isOrientedAngle:    # Rotate to ball.
            direction = Ball.isOrientedAngle
            Robot.rotate(direction * orient_ball_speed)
        else:
            return 'GetToBall'
    else:
        return 'FindBall'


def do_GetToBall():
    global stuck_counter
    if stuck_counter >= stuck_counter_max:
        stuck_counter = 0
        return 'Stuck'
    stuck_counter = stuck_counter + 1

    if Ball.isDetected:
        if not Ball.isOrientedAngle:
            return 'OrientToBall'
        else:
            if Ball.isNear != 0:
                direction = Ball.isNear
                Robot.move_to(Ball, (move_to_ball_speed * direction)
                return 'GetToBall'
            else:
                return 'RotateAroundBall'
    else:
        return 'FindBall'


def do_RotateAroundBall():
    global stuck_counter
    if stuck_counter >= stuck_counter_max:
        stuck_counter = 0
        return 'Stuck'
    stuck_counter = stuck_counter + 1

    if Ball.isDetected:
        if not Ball.isOrientedAngle:    # Not oriented to ball
            return 'OrientToBall'
        else:
            if Ball.isNear != 0:    # Far from Ball
                direction = Ball.isNear
                Robot.move_to(Ball, (direction * move_to_ball_speed))
                return 'GetToBall'
            else:
                if Basket.isDetected:
                    if Basket.isOrientedAngle:  # Requirements fulfill, time to throw
                        return 'Throw'
                    else:   # Rotate with direction to basket
                        direction = Basket.isOrientedAngle
                        Robot.rotate(direction * rotate_around_ball_speed_slow)
                        return 'RotateAroundBall'
                else:   # Rotate around ball at normal speed until basket is found
                    Robot.rotate(rotate_around_ball_speed_normal)
                    return 'RotateAroundBall'
    else:   # We lost the ball
        return 'FindBall'


def do_Throw():
    global throwing_counter
    if Basket.isDetected:
        if throwing_counter < throwing_counter_max:
            Robot.throw_to(Basket)
            Robot.move_to(Basket, move_throw_speed)
            throwing_counter = throwing_counter + 1
            return 'Throw'
        else:
            throwing_counter = 0
            Robot.stop_throw()
            Robot.stop_move()
            return 'Initiate'

    else:
        throwing_counter = 0
        Robot.stop_throw()
        Robot.stop_move()
        return 'Initiate'


def do_Stuck():
    if not Basket.isDetected:
        Robot.rotate(find_basket_speed)
        return 'Stuck'
    else:
        if Basket.isOrientedAngle:
            if Basket.isNear != 0:
                direction = Basket.isNear
                Robot.move_to(Basket, (direction * move_to_basket_stuck_speed))
                return 'Stuck'
            else:
                return 'Initiate'
        else:
            direction = Basket.isOrientedAngle
            Robot.rotate(direction * find_basket_speed)
            return 'Stuck'


def execute_routine(cur_routine):
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
    elif cur_routine == 'RotateAroundBall':
        return do_RotateAroundBall()
    elif cur_routine == 'Throw':
        return do_Throw()
    elif cur_routine == 'Stuck':
        return do_Stuck()


"""
How to implement.
theBall and theBasket object has the following properties:
    .isDetected         True if seen, False if unseen
    .x                  X position
    .y                  Y postion
    .d                  Distance
    .angle              Angle from camera midpoint
    .isOrientedPixel    0 if oriented. 1 is to right, 1 is to left, False if not .isDetected
    .isOrientedAngle    0 if oriented. 1 is to right, 1 is to left, False if not .isDetected
    .isNear             0 if in the range, 1 is far, -1 is too near, False if not .isDetected                    

"""


if __name__ == '__main__':
    rospy.init_node('state_machine')
    # Initialize objects
    Ball = theBall()
    Basket = theBasket()
    Robot = theRobot()
    # Line = theLine()

    # Subscription to update values
    line_subscriber = rospy.Subscriber('/line', LineLocation, line_callback, queue_size=1)
    ball_subscriber = rospy.Subscriber('/ball', Depth_BallLocation, ball_callback, queue_size=1)
    basket_subscriber = rospy.Subscriber('/basket', Depth_BasketLocation, basket_callback, queue_size=1)
    referee_subscriber = rospy.Subscriber('/referee', Ref_Command, referee_callback, queue_size=1)

    # Execution
    routine = 'Pause'

    # Stuck counter
    stuck_counter = 0
    stuck_counter_max = 300

    # Throwing counter
    throwing_counter = 0
    throwing_counter_max = 300

    # Movement_variables (-1 to 1)
    find_speed = 0.20
    orient_ball_speed = 0.15
    move_to_ball_speed = 0.18
    move_to_basket_stuck_speed = 0.24
    find_basket_speed = 0.16    # Speed at stuck
    rotate_around_ball_speed_normal = 0.18
    rotate_around_ball_speed_slow = 0.1
    move_throw_speed = .24

    while not rospy.is_shutdown:
        routine = execute_routine(routine)
        rospy.spin()
