#!/usr/bin/env python3

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
from std_msgs.msg import String
from lebot.msg import Wheel
from movement.findBall import findBall as fball
from movement.approachBall import approachBall
from movement.findBasket import findBasket as fbasket
from movement.approachThrow import approachThrow
from movement.throwerCalculation import thrower_calculation
from transfCamCoord import transfCamCoord as tcc
from calcAngle import calc_angle
from lebot.msg import Depth_BallLocation
from lebot.msg import Depth_BasketLocation
from lebot.msg import Ref_Command
from lebot.msg import Thrower

# ______________________________________________________________________________________________________________________

class Logic:
    def __init__(self):
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

        self.ball_x, self.ball_y, self.ball_d = 0, 0, 0
        self.basket_x, self.basket_y, self.basket_d = 0, 0, 0

        self.current_state = None


    def execute_state(self, state):
        self.pub_state_string()

        if self.counter >= 100:
            self.current_state = 'Standby'
            self.counter = 0
            return

        if state == 'Pause':
            self.counter = 0
            return

        if state == 'Standby':  # Changes to findBall state.
            findBall = self.standby_action()
            if findBall:
                self.current_state = 'FindBall'
                self.counter = 0
            self.counter = self.counter + 1
            return

        elif state == 'FindBall':   # Rotation until a ball is detected.
            getToBall = self.find_ball_action()
            if getToBall:
                self.current_state = 'GetToBall'
                self.counter = 0

            self.counter = self.counter + 1
            return

        elif state == 'GetToBall':  # Move towards ball until a certain distance
            atBall = self.get_to_ball_action()
            if atBall:
                self.current_state = 'ImAtBall'
                self.counter = 0

            self.counter = self.counter + 1
            return

        elif state == 'ImAtBall':   # Rotate around ball until basket is found
            haveBasket = self.im_at_ball_action()
            if haveBasket:
                self.current_state = 'Go'
                self.counter = 0
            self.counter = self.counter + 1
            return

        elif state == 'Go':     # Move and set thrower speed until ball out of range.
            haveThrow = self.go_action()
            if haveThrow:
                self.current_state = 'FindBall'
                self.counter = 0
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
        self.state_string.data = str(self.current_state)
        self.state_pub.publish(self.state_string)

    # Actions to perform at each state -------------------
    def standby_action(self):
        return True

    def find_ball_action(self):
        if (self.ball_x == 0 and self.ball_y == 0) or self.ball_d == 0:
            isBallFound = False
            moveValues = fball(isBallFound)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False
        else:
            isBallFound = True
            moveValues = fball(isBallFound)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return True

    def get_to_ball_action(self):
        angle = calc_angle(self.ball_x)
        xP, yP = tcc(self.ball_d, angle)
        if self.ball_d < 182.5:
            moveValues = (xP, yP)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return True

        elif 20 < self.ball_d < 182.5:
            moveValues = approachBall(xP, yP)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False

        else:
            moveValues = approachBall(xP, yP)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False

    def im_at_ball_action(self):
        if (self.basket_x == 0 and self.basket_y == 0) or self.basket_d == 0:
            isBasketFound = False
            moveValues = fbasket(isBasketFound)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False
        else:
            isBasketFound = True
            moveValues = fball(isBasketFound)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return True

    def go_action(self):
        if (self.ball_x == 0 and self.ball_y == 0) or (self.ball_d < 250):   # Ball got lost or ball too far
            return True
        elif (self.basket_x == 0 and self.basket_y == 0) or self.basket_d == 0:    # Basket got lost
            return True
        else:     # Actual movement and throwing.
            moveValues = approachThrow(self.ball_x, self.ball_y, self.basket_x, self.basket_y)
            throwerValue = thrower_calculation(self.basket_d)
            self.thrower_msg.t1 = int(throwerValue)
            self.throw.publish(self.thrower_msg)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False

    def pause_action(self):
        pass

# ______________________________________________________________________________________________________________________

if __name__ == '__main__':
    rospy.init_node('state_machine')
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    fb = Logic()
    fb.current_state = 'Standby'
    while not rospy.is_shutdown():
        fb.execute_state(fb.current_state)
        rate.sleep()

