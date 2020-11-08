import rospy
import smach
import smach_ros
from additional_msgs.msg import Depth_BallLocation
from additional_msgs.msg import Wheel

from srv.srv import ball_srv, ball_srvResponse


class FINDBALL(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['ball', 'noball'])
        self.ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        self.msg = Depth_BallLocation()
        self.myBall = self.ball_service

    def execute(self, userdata):
        if self.myBall.d > 0:
            return 'ball'
        else:
            return 'noball'


class STANDBY(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        return 'start'


class FINDBASKET(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['basket', 'nobasket'])

    def execute(self):
        return 'basket'


class OFF(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self, userdata):
        print('Going off')
        exit()



def main():
    rospy.init_node('/state_machine')
    # Wait for published services to become available.
    rospy.wait_for_service("/ball_service")
    rospy.wait_for_service('/basket_service')

    # State machine
    sm = smach.StateMachine(outcomes=['game', 'exit'])
    with sm:
        smach.StateMachine.add('FINDBALL', FINDBALL(), transitions={'ball': 'OFF', 'noball': 'OFF'})
        smach.StateMachine.add('FINDBASKET', FINDBASKET(), transition = {'nobasket': 'OFF', 'basket' : 'OFF'})
        smach.StateMachine.add('STANDBY', STANDBY(), transition = {'start':'FINDBASKET'})
        smach.StateMachine.add('OFF', OFF())

    outcome = sm.execute()


if __name__ == '__main__':
    main()