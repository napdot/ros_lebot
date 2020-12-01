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

class FINDBALL(smach.State):
    def __int__(self):
        self.msg = Wheel()
        smach.State.__init__(self, outcomes=['ballFound'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.ball_subscriber = rospy.Suscriber('/ball', Depth_BallLocation, self.ball_callback, queue_size=1)
        self.x, self.y, self.d = 0, 0, 0

    def execute(self):  # execute(self, userdata)
        if (self.x == 0 and self.y == 0) or self.d == 0:
            isBallFound = False
            self.msg.w1, self.msg.w2, self.msg.w3 = fball(isBallFound)
            self.move.publish(self.msg)
        else:
            isBallFound = True
            self.msg.w1, self.msg.w2, self.msg.w3 = fball(isBallFound)
            self.move.publish(self.msg)
            return 'ballFound'

    def ball_callback(self, data):
        self.x, self.y, self.d = data.x, data.y, data.d


class GETTOBALL(smach.State):
    def __int__(self):
        self.msg = Wheel()
        smach.State.__init__(self, outcomes=['atBall'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.ball_subscriber = rospy.Suscriber('/ball', Depth_BallLocation, self.ball_callback, queue_size=1)
        self.x, self.y, self.d = 0, 0, 0

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

    def ball_callback(self, data):
        self.x, self.y, self.d = data.x, data.y, data.d


class STANDBY(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        print('Standby')
        print('Going to start...')
        return 'start'


# ______________________________________________________________________________________________________________________

def main():
    rospy.init_node('state_machine')
    # Wait for published services to become available.
    rospy.wait_for_service("/ball_service")

    # State machine
    sm = smach.StateMachine(outcomes=['OFF', 'pause'])
    with sm:
        smach.StateMachine.add('STANDBY', STANDBY(),
                               transition={'start': 'FINDBALL'})

        # Rotate to find ball, once ball is found then go to GETTOBALL
        smach.StateMachine.add('FINDBALL', FINDBALL(), transition={'ballFound': 'GETTOBALL'})

        # State for approaching ball
        smach.StateMachine.add('GETTOBALL', GETTOBALL(),
                               transition={'atBall': 'STANDBY'})
    outcome = sm.execute()


if __name__ == '__main__':
    main()
