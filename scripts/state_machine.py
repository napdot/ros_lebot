import rospy
import smach
import smach_ros
from msg.msg import Depth_BallLocation
from msg.msg import Wheel
from srv.srv import ball_srv, ball_srvResponse
from srv.srv import basket_srv, basket_srvResponse
from scripts.movement.findBall import findBall as fball
from scripts.movement.findBasket import findBasket as fbasket
from scripts.movement.approachBall import approachBall
from scripts.movement.approachThrow import approachThrow
from scripts.transfCamCoord import transfCamCoord as tcc
from scripts.calcAngle import calc_angle


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


class IMATBALL(smach.State):
    def __int__(self):
        self.msg = Wheel()
        smach.State.__init__(self, outcomes=['basketFound', 'noBasket'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)

    def execute(self):  # execute(self, userdata)
        x, y, d = self.findbasket_service()
        if x == 0 and y == 0 or d = 0:
            return 'noBasket'
        else:
            return 'basketFound'

    def findbasket_service(self):
        basket_service = rospy.ServiceProxy('/basket_service', basket_srv)
        x, y, d = basket_service
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


class ROTATEAROUNDBALL(smach.State):
    def __int__(self):
        self.msg = Wheel()
        smach.State.__init__(self, outcomes=['basketFound'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)

    def execute(self):
        x, y, d = self.findbasket_service()
        if x != 0 and y != 0 or d !=0:
            isBasketFound = False
            self.msg.w1, self.msg.w2, self.msg.w3 = fbasket(isBasketFound)
            self.move.publish(self.msg)
        else:
            isBasketFound = True
            self.msg.w1, self.msg.w2, self.msg.w3 = fbasket(isBasketFound)
            self.move.publish(self.msg)
            return 'basketFound'

    def findbasket_service(self):
        basket_service = rospy.ServiceProxy('/basket_service', basket_srv)
        x, y, d = basket_service
        return x, y, d


class STANDBY(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['start, goOff'])

    def execute(self, userdata):
        print('Standby')
        print('Going to start...')
        return 'start'


class GO(smach.State):
    def __init__(self):
        self.msg = Wheel()
        smach.State.__init__(self, outcomes=['haveThrow'])
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)

    def execute(self):
        bx, by, bd = self.findball_service()
        gx, gy, gd = self.findbasket_service()
        bangle = calc_angle(bx)
        gangle = calc_angle(gx)
        bxP, byP = tcc(bd, bangle)
        gxP, gyP = tcc(gd, gangle)
        approachThrow(bxP, byP, gxP, gyP)

        if bx == 0 and by == 0 or bd == 0:
            return 'haveThrow'
        if gx == 0 and gy == 0 or bd == 0:
            return 'haveThrow'

    def findball_service(self):
        ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        x, y, d = ball_service
        return x, y, d

    def findbasket_service(self):
        basket_service = rospy.ServiceProxy('/basket_service', basket_srv)
        x, y, d = basket_service
        return x, y, d


class OFF(smach.State):
    def __int__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self):
        print('Going off')
        rospy.signal_shutdown('OFF state and exit.')
        return 'exit'


class PAUSE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self):
        rospy.Subscriber('/refereesignal')


class READY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noBall', 'isReady'])

    def execute(self):
        bx, by, bd = self.findball_service()
        if bx == 0 and by == 0 or bd == 0:
            return 'noBall'
        gx, gy, gd = self.findbasket_service()
        if gx != 0 and gy != 0 or gd != 0:
            return 'isReady'

    def findball_service(self):
        ball_service = rospy.ServiceProxy('/ball_service', ball_srv)
        x, y, d = ball_service
        return x, y, d

    def findbasket_service(self):
        basket_service = rospy.ServiceProxy('/basket_service', basket_srv)
        x, y, d = basket_service
        return x, y, d


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
        smach.StateMachine.add('STANDBY', STANDBY(),
                               transition={'start': 'READY', 'goOFF': "OFF"})

        # READY is a check phase that checks if everything is ready (in the game scope) to go ahead and throw.
        # Check if Ball and Basket, if yes, then go to SET. else FINDBALL.
        smach.StateMachine.add('READY', READY(),
                               transition={'noBall': 'FINDBALL', 'isReady': 'GO'})

        # Rotate to find ball, once ball is found then go to GETTOBALL
        smach.StateMachine.add('FINDBALL', FINDBALL(), transition={'ballFound': 'GETTOBALL'})

        # State for approaching ball
        smach.StateMachine.add('GETTOBALL', GETTOBALL(),
                               transition={'atBall': 'IMATBALL'})

        # Once we get to Ball check if there's a basket. If there is, then READY. If no basket, ROTATEAROUNDBALL
        smach.StateMachine.add('IMATBALL', STANDBY(),
                               transition={'noBasket': 'ROTATEAROUNDBALL', 'basketFound': 'READY'})

        # Rotate around ball until basketFound and move to READY
        smach.StateMachine.add('ROTATEAROUNDBALL', ROTATEAROUNDBALL(),
                               transition={'basketFound': 'READY'})

        # In Go we initiate thrower motor and move forward.
        # If no ball in thrower visual range, then we can assume we have thrown and go to READY.
        smach.StateMachine.add('GO', GO(),
                               transition={'haveThrow': 'READY'})
        smach.StateMachine.add('PAUSE', PAUSE())

    outcome = sm.execute()


if __name__ == '__main__':
    global spd, minBallRangeThrow
    spd = 100
    minBallRangeThrow = 182.5
    main()
