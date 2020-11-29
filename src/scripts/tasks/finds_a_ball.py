#!/usr/bin/env python3

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
import smach
from lebot.msg import Wheel
from lebot.srv import ball
from movement.findBall import findBall as fball
from movement.approachBall import approachBall
from transfCamCoord import transfCamCoord as tcc
from calcAngle import calc_angle


# ______________________________________________________________________________________________________________________

class FINDBALL(smach.State):
    def __int__(self):
        self.msg = Wheel()
        smach.State.__init__(self, outcomes=['ballFound'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)

    def execute(self):  # execute(self, userdata)
        x, y, d = self.findball_service()
        if x != 0 and y != 0:
            isBallFound = False
            self.msg.w1, self.msg.w2, self.msg.w3 = fball(isBallFound)
            self.move.publish(self.msg)
        else:
            isBallFound = True
            self.msg.w1, self.msg.w2, self.msg.w3 = fball(isBallFound)
            self.move.publish(self.msg)
            return 'ballFound'

    def findball_service(self):
        ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        x, y, d = ball_service
        return x, y, d


class GETTOBALL(smach.State):
    def __int__(self):
        self.msg = Wheel()
        smach.State.__init__(self, outcomes=['atBall'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)

    def execute(self):
        x, y, d = self.findball_service()
        angle = calc_angle(x)
        xP, yP = tcc(d, angle)
        if d > minBallRangeThrow:
            self.msg.w1, self.msg.w2, self.msg.w3 = approachBall(xP, yP)
            self.move.publish(self.msg)

        else:
            self.msg.w1, self.msg.w2, self.msg.w3 = approachBall(xP, yP)
            self.move.publish(self.msg)
            return 'atBall'

    def findball_service(self):
        ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        x, y, d = ball_service
        return x, y, d


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
