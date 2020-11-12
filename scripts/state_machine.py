import rospy
import smach
import smach_ros
from msg.msg import Depth_BallLocation
from msg.msg import Wheel
from srv.srv import ball_srv, ball_srvResponse


# ______________________________________________________________________________________________________________________

class FINDBALL(smach.State):
    def __int__(self):
        self.isOrientedBall = False
        self.isNearBall = False
        self.msg = Wheel()
        smach.State.__init__(self, outcomes=['ball', 'noball'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)

    def execute(self):  # execute(self, userdata)
        x, y, d = self.findball_service()
        if x == 0 and y == 0:
            return 'noball'

        self.isOrientedBall = self.orient_to_ball(x)

        if self.isOrientedBall:
            self.isNearBall = self.move_to_ball(d)

        if self.isNearBall:
            print('Oriented and near')
            return 'ball'

    def findball_service(self):
        ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        x, y, d = ball_service
        return x, y, d

    def move_to_ball(self, d):
        if d > 30:
            self.msg.w1, self.msg.w2, self.msg.w3 = spd, spd, spd
            self.move.publish(self.msg)
            return False
        else:
            return True

    def orient_to_ball(self, x):
        if (center_point - ball_freedom) < x < (center_point + ball_freedom):   # Ball is middle
            return True
        elif x < (center_point - ball_freedom):     # Ball is left
            self.msg.w1, self.msg.w2, self.msg.w3 = spd, -spd, -spd
            self.move.publish(self.msg)
            return False
        elif (center_point + ball_freedom) < x:     # Ball is right
            self.msg.w1, self.msg.w2, self.msg.w3 = -spd, spd, spd
            self.move.publish(self.msg)
            return False

class GETTOBALLWITHBASKET(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['ballFound', 'noBallFound'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)

    def execute(self):  # execute(self, userdata)
        x, y, d = self.findball_service()
        if x != 0 and y != 0:
            return 'ballFound'

        if self.counter < 100:
            if self.negRot:
                self.rotate_to(1, 1, 1)
                self.counter = self.counter + 1
                if self.counter == 50:
                    negRot = False
            else:
                self.rotate_to(-1, -1, -1)
                self.counter = self.counter + 1
        else:
            print('100 iteration. ball no found')
            return 'noBallFound'

class NOBALL(smach.State):
    def __int__(self):
        self.counter = 0
        self.negRot = True
        smach.State.__init__(self, outcomes=['ballFound', 'noBallFound'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)

    def execute(self):  # execute(self, userdata)
        x, y, d = self.findball_service()
        if x != 0 and y != 0:
            return 'ballFound'

        if self.counter < 100:
            if self.negRot:
                self.rotate_to(1, 1, 1)
                self.counter = self.counter + 1
                if self.counter == 50:
                    negRot = False
            else:
                self.rotate_to(-1, -1, -1)
                self.counter = self.counter + 1
        else:
            print('100 iteration. ball no found')
            return 'noBallFound'

    def findball_service(self):
        ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        x, y, d = ball_service
        return x, y, d

    def rotate_to(self, a, b, c):
        self.msg.w1, self.msg.w2, self.msg.w3 = a*spd, b*spd, c*spd
        self.move.publish(self.msg)


class STANDBY(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['toStart', 'toOFF'])

    def execute(self, userdata):
        return 'start'


class FINDBASKET(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['basket', 'nobasket'])

    def execute(self):

        print('State: FINDBASKET')

        return 'basket'


class STANDBY(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        print('Standby')
        print('Going to start...')
        return 'start'


class BALLBASKET(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['basket', 'nobasket', 'ballNotInThrower'])
        self.

    def execute(self):
        ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        x, y, d = self.findball_service()

        isBallInThrower = self.checkBallinThrower(x, y, d)
        if not isBallInThrower:
            return 'ballNotInThrower'




    def findball_service(self):
        ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        x, y, d = ball_service
        return x, y, d


    def checkBallinThrower(self, x, y, d):
        if:
            return True
        else:
            return False



class OFF(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self, userdata):
        print('Going off')
        rospy.signal_shutdown('OFF state and exit.')


class PAUSE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self, userdata):
        rospy.Subscriber('/refereesignal')


# ______________________________________________________________________________________________________________________

def main():
    rospy.init_node('/state_machine')
    # Wait for published services to become available.
    rospy.wait_for_service("/ball_service")
    rospy.wait_for_service('/basket_service')


    # State machine
    sm = smach.StateMachine(outcomes=['OFF', 'pause'])
    with sm:
        # Wait for signal to start or Off somehow
        smach.StateMachine.add('STANDBY', STANDBY(), transition={'start': 'READY', 'goOFF': "OFF"})
        # READY is a check phase that checks if everything is ready (in the game scope) to go ahead and throw.
        # Check if Ball and Basket, if yes, then go to SET. If noBall, FINDBALL. If no basket, GETTOBALL (NOTE: noBallTakes priority regardless if basket or noBasket.)
        smach.StateMachine.add('READY', READY(), transition={'noBall': 'FINDBALL', 'noBasket': "GETTOBALL", 'isReady' : 'SET'})
        # Rotate to find ball, once ball is found then go to GETTOBALL
        smach.StateMachine.add('FINDBALL', FINDBALL(), transition={'ballFound': 'GETTOBALL'})
        # Once we get to Ball check if there's a basket. If there is, then READY. If no basket, ROTATEAROUNDBALL
        smach.StateMachine.add('GETTOBALL', STANDBY(), transition={'noBasket': 'ROTATEAROUNDBALL', 'basketFound': 'READY'})
        # Rotate around ball until basketFound and move to READY
        smach.StateMachine.add('ROTATEAROUNDBALL', ROTATEAROUNDBALL(), transition={'basketFound': 'READY'})
        # In set we move to correct direction and distance from ball. Once set up, we go to GO.
        smach.StateMachine.add('SET', SET(), transition={'letsGo': 'GO'})
        # In Go we initiate thrower motor and move forward. If no ball in thrower visual range, then we can assume we have thrown and go to READY.
        smach.StateMachine.add('GO', GO(), transition={'haveThrow': 'READY'})









        smach.StateMachine.add('STANDBY', STANDBY(), transition={'start': 'FINDBALL', 'goOFF': "OFF"})
        smach.StateMachine.add('FINDBALL', FINDBALL(), transitions={'ball': 'BALLBASKET', 'noBall': 'NOBALL'})
        smach.StateMachine.add('NOBALL', NOBALL(), transitions={'ball': 'FINDBALL', 'noball': 'STANDY'})
        smach.StateMachine.add('GETTOBALLWITHBASKET', GETTOBALLWITHBASKET(), transition={'readyToThrow': 'THROW', 'noBasket': 'ROTAROUNDBALL', 'noBall': 'FINDBALL'})
        smach.StateMachine.add('PAUSE', PAUSE())

    outcome = sm.execute()


if __name__ == '__main__':
    global ball_freedom, center_point, spd
    ball_freedom = 30
    center_point = 1920 / 2
    spd = 10
    main()
