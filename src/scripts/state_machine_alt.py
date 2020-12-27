#!/usr/bin/env python3

import os, sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
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
from lebot.msg import Depth_BallLocation
from lebot.msg import Depth_BasketLocation
from lebot.msg import Ref_Command
from lebot.msg import Thrower


# ______________________________________________________________________________________________________________________

class Logic:
    def __init__(self, min_dist, node_rate):
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.throw = rospy.Publisher('/thrower_values', Thrower, queue_size=1)
        self.state_pub = rospy.Publisher('/lebot_state', String, queue_size=1)
        self.ball_subscriber = rospy.Subscriber('/ball', Depth_BallLocation, self.ball_callback, queue_size=1)
        self.basket_subscriber = rospy.Subscriber('/basket', Depth_BasketLocation, self.basket_callback, queue_size=1)
        self.referee_subscriber = rospy.Subscriber('/referee', Ref_Command, self.referee_callback, queue_size=1)

        self.state_string = String()

        self.thrower_msg = Thrower()
        self.msg = Wheel()
        self.msg.w1, self.msg.w2, self.msg3 = 0, 0, 0

        self.last_state = None
        self.counter = 0
        self.throwing_counter = 0  # Must figure a way to avoid using it on GO state

        self.ball_x, self.ball_y, self.ball_d = 0, 0, 0
        self.basket_x, self.basket_y, self.basket_d = 0, 0, 0

        self.current_state = None

        self.min_ball_dist = min_dist
        self.orientation_offset = 15
        self.distance_offset = 40
        self.rate = node_rate

    def execute_state(self, state):
        self.pub_state_string()

        if self.counter >= (self.rate * 3) and state != "Pause":
            self.current_state = 'Standby'
            self.counter = 0

        if state == 'Pause':
            self.counter = 0
            return

        if state == 'Standby':  # Changes to findBall state.
            next = self.standby_action()
            if next:
                self.current_state = 'FindBall'
                self.counter = 0
                return
            self.counter = self.counter + 1
            return

        elif state == 'FindBall':  # Rotation until a ball is detected.
            next = self.find_ball_action()
            if next:
                self.current_state = 'GetToBall'
                self.counter = 0
                return
            self.counter = self.counter + 1
            return

        elif state == 'GetToBall':  # Move towards ball until a certain distance
            next = self.get_to_ball_action()
            if next:
                self.current_state = 'ImAtBall'
                self.counter = 0
                return
            self.counter = self.counter + 1
            return

        elif state == 'ImAtBall':  # Rotate around ball until basket is found
            next = self.im_at_ball_action()
            if next:
                self.current_state = 'Align'
                self.counter = 0
                return
            self.counter = self.counter + 1
            return

        elif state == 'Align':
            next = self.align_action()
            if next:
                self.current_state = 'Go'
                self.counter = 0
                return
            self.counter = self.counter + 1


        elif state == 'Go':  # Move and set thrower speed until ball out of range.
            next = self.go_action()
            if next:
                self.current_state = 'Standby'
                self.counter = 0
                return
            self.counter = self.counter + 1
            return

    # Callback from subscriptions
    def ball_callback(self, data):
        self.ball_x, self.ball_y, self.ball_d = data.x, data.y, data.d

    def basket_callback(self, data):
        self.basket_x, self.basket_y, self.basket_d = data.x, data.y, data.d

    def referee_callback(self, data):
        command_string = data.command
        if command_string == 'pause':
            self.current_state = 'Pause'

        elif command_string == 'resume':
            self.current_state = 'Standby'
            self.execute_state(self.current_state)

    def pub_state_string(self):
        self.state_string.data = str(self.counter) + " : " + str(self.current_state)
        self.state_pub.publish(self.state_string)

    """
     ___Actions to perform at each state___
     Return True to proceed to next state
     Return False to stay on same state
     Return False and self.current = new_state to change to specific state (Remember to reset counter)
    """

    def standby_action(self):
        return True

    def find_ball_action(self):
        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # No ball in sight
            moveValues = fball()
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False
        else:  # Ball in sight.
            return True

    def get_to_ball_action(self):
        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:
            self.current_state = 'FindBall'
            self.counter = 0
            return False

        if abs(self.ball_x) > self.orientation_offset:
            moveValues = orient(self.ball_x)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False

        else:
            angle = calc_angle(self.ball_x, self.ball_d)
            xP, yP = tcc(self.ball_d, angle)
            if self.ball_d > self.min_ball_dist:  # Not yet near ball
                moveValues = approachBall(xP, yP)
                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                self.move.publish(self.msg)
                return False

            elif 1 < self.ball_d < self.min_ball_dist:  # Ball located and near
                return True

    def im_at_ball_action(self):
        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # Ball lost
            self.current_state = 'FindBall'
            self.counter = 0
            return False

        if abs(self.ball_x) > self.orientation_offset:
            moveValues = orient(self.ball_x)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False

        elif (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:  # Finding basket
            moveValues = fbasket()
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False

        else:  # Basket and ball found
            return True

    def go_action(self):
        # Figure some efficient way.
        if self.throwing_counter > self.rate:
            self.throwing_counter = 0
            return True

        if abs(self.basket_x) > self.orientation_offset:
            moveValues = orient(self.ball_x)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False

        else:
            moveValues = approachThrow(self.ball_x, self.ball_y, self.basket_x, self.basket_y)
            throwerValue = thrower_calculation(self.basket_d)
            self.thrower_msg.t1 = int(throwerValue)
            self.throw.publish(self.thrower_msg)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            self.throwing_counter = self.throwing_counter + 1
            return False

    def align_action(self):
        if (not (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0) and not ((self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0):
            ball_angle = calc_angle(self.ball_x, self.ball_d)
            ball_xP, ball_yP = tcc(self.ball_d, ball_angle)
            basket_angle = calc_angle(self.basket_x, self.basket_d)
            basket_xP, basket_yP = tcc(self.basket_d, basket_angle)
            coord = align_throw(ball_xP, ball_yP, basket_xP, basket_yP)
            if abs(coord[0]) < self.distance_offset and abs(coord[1]) < self.distance_offset:
                return True
            moveValues = ots(coord[0], coord[1])
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            return False
        self.current_state = 'FindBall'
        self.counter = 0
        return False

    def go_action_alt(self):  # Doesn't work because at some point we lost the ball when too near...
        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # Ball lost
            self.current_state = 'FindBall'
            self.counter = 0
            return False

        elif self.ball_d > (self.min_ball_dist + self.distance_offset):  # Ball too far
            self.current_state = 'GetToBall'
            self.counter = 0
            return False

        elif (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:  # Basket got lost
            self.current_state = 'ImAtBall'
            self.counter = 0
            return False

        else:  # Actual movement and throwing.
            moveValues = approachThrow(self.ball_x, self.ball_y, self.basket_x, self.basket_y)
            throwerValue = thrower_calculation(self.basket_d)
            self.thrower_msg.t1 = int(throwerValue)
            self.throw.publish(self.thrower_msg)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False

    def pause_action(self):
        pass

    def wheel_stop(self):
        self.msg.w1, self.msg.w2, self.msg.w3 = int(0), int(0), int(0)
        self.move.publish(self.msg)


# ______________________________________________________________________________________________________________________

# No ball = (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:
# No basket = (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:

# rostopic pub /referee lebot/Ref_Command "{'command':'resume'}"

if __name__ == '__main__':
    rospy.init_node('state_machine')
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    fb = Logic(min_dist=450, node_rate=myRate)
    fb.current_state = 'Pause'
    while not rospy.is_shutdown():
        fb.execute_state(fb.current_state)
        rate.sleep()
