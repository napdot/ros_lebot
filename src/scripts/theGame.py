#!/usr/bin/env python3

import os, sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
import numpy as np
from std_msgs.msg import String
from lebot.msg import Wheel
from movement.orientObject import orient
from movement.findBall import findBall as fball
from movement.approachBall import approachBall
from movement.findBasket import findBasket as fbasket
from movement.approachThrow import approachThrow
from movement.throwerCalculation import thrower_calculation
from movement.alignThrow import align_throw
from omni import omni_to_serial as ots
from transfCamCoord import transfCamCoord as tcc
from calcAngle import calc_angle
from calcAngleCameraOrientation import calc_angle_cam
from lebot.msg import Depth_BallLocation
from lebot.msg import Depth_BasketLocation
from lebot.msg import Ref_Command
from lebot.msg import Thrower
from lebot.msg import LineLocation


# Message
move_msg = Wheel()


# Variables
ball_x, ball_y, ball_d = 0, 0, 0
basket_x, basket_y, basket_d = 0, 0, 0
line_x1, line_y1, line_x2, line_x3


# Topics to use
move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
throw = rospy.Publisher('/thrower_values', Thrower, queue_size=1)
state_pub = rospy.Publisher('/lebot_state', String, queue_size=1)
ball_subscriber = rospy.Subscriber('/ball', Depth_BallLocation, ball_callback, queue_size=1)
basket_subscriber = rospy.Subscriber('/basket', Depth_BasketLocation, basket_callback, queue_size=1)
referee_subscriber = rospy.Subscriber('/referee', Ref_Command, referee_callback, queue_size=1)
line_subscriber = rospy.Subscriber('/line', LineLocation, line_callback, queue_size=1)



def ball_callback(data):
    global ball_x, ball_y, ball_d
    ball_x, ball_y, ball_d = data.x, data.y, data.d


def basket_callback(data):
    global basket_x, basket_y, basket_d
    basket_x, basket_y, basket_d = data.x, data.y, data.d


def line_callback(data):
    global line_x1, line_y1, line_x2, line_y2
    line_x1, line_y1, line_x2, line_y2 = data.x1, data.y1, data.x2, data.y2


def isBall():
    if (ball_x == -320 and ball_y == 480) or ball_d == 0:
        return False
    else:
        return True

def isBallNear():
    if ball_d < min_dist_ball:
        return True
    else:
        return False

def isBallOriented():
    angle = calc_angle_cam(ball_x)
    if abs(angle) < ball_orientation_offset:
        return True
    else:
        return False


def whereIsBall():
    if ball_x > 0:
        return 1
    else:
        return 0

def isBasketNear():
    if basket_d < min_dist_basket:
        return True
    else:
        return False

def isBasket():
    if (ball_x == -320 and ball_y == 480) or ball_d == 0:
        return False
    else:
        return True

def isLine():
    if (line_x1 == -320 and line_y1 == 480) or (line_x2 == -320 and line_y2 == 480):
        return False
    else:
        return True








def referee_callback(data):
    command_string = data.command
    if command_string == 'pause':
        current_state = 'Pause'

    elif command_string == 'resume':
        current_state = 'Standby'
        execute_state(current_state)

state = None

def have_ball():
    global idohaveball
    while idohaveball:



def stop_moving_and_throwing():
    global mov_msg
    moveValues = [0, 0, 0]
    msg.w1, msg.w2, msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
    move.publish(msg)
    throwValues = int(0)
    ms


while True:
    if state == 'Pause':

    if state == 'HaveBall':
        idohaveball = True
        have_ball()
        continue

    if state == 'Oriented'"":
        or

    if isBall():
        if isBallOriented():
            if isBallNear():
                state = 'HaveBall'
                continue
            state = 'OrientedBall'
            continue
        state = 'BallFound'
        continue
    else:
        state  = 'NoBall'
        continue













if __name__ == '__main__':
    rospy.init_node('state_machine')
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    fb = Logic(min_dist=370, node_rate=myRate, line_detection=False, stuck_activated=False)
    fb.current_state = 'Pause'
    while not rospy.is_shutdown():
        rospy.spin()
        logic()
        rate.sleep()
