#!/usr/bin/env python3

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
import smach
from lebot.msg import Wheel
from movement.findBall import findBall as fball
from movement.approachBall import approachBall
from transfCamCoord import transfCamCoord as tcc
from calcAngle import calc_angle
from lebot.msg import Depth_BallLocation
from lebot.msg import Depth_BasketLocation

# ______________________________________________________________________________________________________________________

class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start', 'ballFound', 'atBall'])

        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.ball_subscriber = rospy.Subscriber('/ball', Depth_BallLocation, self.ball_callback, queue_size=1)
        self.basket_subscriber = rospy.Publisher('/basket', Depth_BasketLocation, self.basket_callback, queue_size=1)

        self.msg = Wheel()
        self.msg.w1, self.msg.w2, self.msg3 = 0, 0, 0

    def execute(self):
        rospy.loginfo('State')

    def ball_callback(self, data):
        self.ball_x, self.ball_y, self.ball_d = data.x, data.y, data.d

    def basket_callback(self, data):
        self.basket_x, self.basket_y, self.basket_d = data.x, data.y, data.d


class FINDBALL(Foo):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start', 'ballFound', 'atBall'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.ball_subscriber = rospy.Subscriber('/ball', Depth_BallLocation, self.ball_callback, queue_size=1)
        self.basket_subscriber = rospy.Publisher('/basket', Depth_BasketLocation, self.basket_callback, queue_size=1)

        self.msg = Wheel()
        self.msg.w1, self.msg.w2, self.msg3 = 0, 0, 0

    def execute(self):
        if (self.x == 0 and self.y == 0) or self.d == 0:
            isBallFound = False
            self.msg.w1, self.msg.w2, self.msg.w3 = fball(isBallFound)
            self.move.publish(self.msg)
        else:
            isBallFound = True
            self.msg.w1, self.msg.w2, self.msg.w3 = fball(isBallFound)
            self.move.publish(self.msg)
            return 'ballFound'


class GETTOBALL(Foo):
    def execute(self):
        angle = calc_angle(self.x)
        xP, yP = tcc(self.d, angle)
        if d > minBallRangeThrow:
            self.msg.w1, self.msg.w2, self.msg.w3 = approachBall(xP, yP)
            self.move.publish(self.msg)

        else:
            self.msg.w1, self.msg.w2, self.msg.w3 = approachBall(xP, yP)
            self.move.publish(self.msg)
            return 'atBall'


class STANDBY(Foo):
    def execute(self):
        rospy.loginfo('Standby - Now starting....')
        return 'start'


class OFF(Foo):
    def execute(self):
        rospy.loginfo('OFF - finsihing')
        rospy.signal_shutdown('find_a_ball done')
        return 'OFF'


# ______________________________________________________________________________________________________________________

def main():
    rospy.init_node('state_machine')

    # State machine
    sm = smach.StateMachine(outcomes=['OFF', 'pause'])
    with sm:
        smach.StateMachine.add('STANDBY', STANDBY(),
                               transitions={'start': 'FINDBALL', 'ballFound': 'STANDBY', 'atBall': 'STANDBY'})

        # Rotate to find ball, once ball is found then go to GETTOBALL
        smach.StateMachine.add('FINDBALL', FINDBALL(),
                               transitions={'start': 'FINDBALL', 'ballFound': 'GETTOBALL', 'atBall': 'FINDBALL'})
        # State for approaching ball
        smach.StateMachine.add('GETTOBALL', GETTOBALL(),
                               transitions={'atBall': 'STANDBY'})
    outcome = sm.execute()


if __name__ == '__main__':
    main()
