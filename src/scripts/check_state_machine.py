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
from movement.fb2 import findBasketSlow as fbasket2
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


# ______________________________________________________________________________________________________________________

class Logic:
    def __init__(self, min_dist, node_rate, line_detection, stuck_activated):
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.throw = rospy.Publisher('/thrower_values', Thrower, queue_size=1)
        self.state_pub = rospy.Publisher('/lebot_state', String, queue_size=1)
        self.ball_subscriber = rospy.Subscriber('/ball', Depth_BallLocation, self.ball_callback, queue_size=1)
        self.basket_subscriber = rospy.Subscriber('/basket', Depth_BasketLocation, self.basket_callback, queue_size=1)
        self.referee_subscriber = rospy.Subscriber('/referee', Ref_Command, self.referee_callback, queue_size=1)
        self.line_subscriber = rospy.Subscriber('/line', LineLocation, self.line_callback, queue_size=1)

        self.state_string = String()

        self.thrower_msg = Thrower()
        self.msg = Wheel()
        self.msg.w1, self.msg.w2, self.msg3 = 0, 0, 0

        self.last_state = None
        self.counter = 0
        self.throwing_counter = 0  # Must figure a way to avoid using it on GO state

        self.ball_x, self.ball_y, self.ball_d = 0, 0, 0
        self.basket_x, self.basket_y, self.basket_d = 0, 0, 0
        self.line_x1, self.line_y1, self.line_x2, self.line_y2 = 0, 0, 0, 0

        self.current_state = None

        self.detect_line = line_detection

        self.min_ball_dist = min_dist

        """
        Orientation offsets must follow these conditions:
        find < rot < mov
        throw is independent but ideally < find
        Small find makes it harder to start moving to balls, especially if they are far.
        Small mov, makes it run much slower due to more need to correct orientation to balls.
        Small rot, makes it so that we correct our orientation to more frequently while finding baskets. However, we
        would want it to be fairly small for better throwing.
        Small throw would be the most ideal. Tradeoff would only be speed.
        """

        self.orientation_offset_mov = 10 * np.pi / 180
        self.orientation_offset_find = 7 * np.pi / 180
        self.orientation_offset_rot = 90 * np.pi / 180
        self.orientation_offset_throw = 4 * np.pi / 180
        self.orientation_offset_pre_throw = 15 * np.pi / 180
        self.orientation_offset_throw_ball = 7 * np.pi / 180
        self.orientation_offset_stuck = 15 * np.pi / 180
        self.distance_offset = 40
        self.rate = node_rate
        self.throw_duration = 2.4   # in seconds

        self.rot = 1

        self.stuck_at_state = False
        self.can_get_stuck = stuck_activated
        self.stuck_counter = 0
        self.stuck_max = node_rate * 10

    def stop_wheel(self):
        moveValues = [0, 0, 0]
        self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
        self.move.publish(self.msg)

    def stop_thrower(self):
        self.throwing_counter = 0
        self.thrower_msg.t1 = int(0)
        self.throw.publish(self.thrower_msg)

    def execute_state(self, state):
        self.pub_state_string()

        if self.counter >= (self.rate * 4) and state != "Pause":
            self.stop_wheel()
            self.stop_thrower()
            self.last_state = self.current_state
            self.current_state = 'Standby'
            self.counter = 0
            if self.can_get_stuck and state != "ImAtBall":
                self.stuck_at_state = True
                self.current_state = 'Stuck'
                return

        if state == 'Pause':
            self.stop_wheel()
            self.stop_thrower()
            self.counter = 0
            return

        if state == 'Stuck':
            if self.stuck_counter > self.stuck_max:  # Couldn't get unstuck
                self.current_state = 'Standby'
                self.stuck_at_state = False
                self.counter = 0
                self.stuck_counter = 0
                self.last_state = 'Standby'
                rospy.logwarn("Couldn't get unstuck")
                return

            unstuck = self.stuck_at_state_action()
            if unstuck:
                self.stuck_at_state = False
                self.current_state = 'Standby'
                self.stuck_counter = 0
                self.counter = 0
                self.stuck_counter = 0
                self.last_state = 'Standby'
                return
            self.stuck_counter = self.stuck_counter + 1
            return

        if state == 'Standby':  # Changes to findBall state.
            next = self.standby_action()
            self.stop_thrower()
            if next:
                self.current_state = 'FindBall'
                self.counter = 0
                self.stop_wheel()
                return
            self.counter = self.counter + 1
            return

        elif state == 'FindBall':  # Rotation until a ball is detected.
            next = self.find_ball_action()
            if next:
                self.current_state = 'GetToBall'
                # self.current_state = 'Pause'
                self.counter = 0
                # self.stop_wheel()
                return
            self.counter = self.counter + 1
            return

        elif state == 'GetToBall':  # Move towards ball until a certain distance
            next = self.get_to_ball_action()
            self.stop_thrower()
            if next:
                self.current_state = 'ImAtBall'
                # self.current_state = 'Pause'
                self.counter = 0
                # self.stop_wheel()
                return
            self.counter = self.counter + 1
            return

        elif state == 'ImAtBall':  # Rotate around ball until basket is found
            self.stop_thrower()
            next = self.im_at_ball_action()
            if next:
                self.current_state = 'Go'
                # self.current_state = 'Pause'
                self.counter = 0
                # self.stop_wheel()
                return
            self.counter = self.counter + 1
            return

        elif state == 'Align':
            next = self.align_action()
            if next:
                self.current_state = 'Pause'
                self.counter = 0
                return
            self.counter = self.counter + 1
            return

        elif state == 'Go':  # Move and set thrower speed until ball out of range.
            self.stop_thrower()
            next = self.go_action()
            if next:
                self.current_state = 'Throw'
                # self.current_state = 'Pause'
                self.counter = 0
                self.stop_wheel()
                return
            self.counter = self.counter + 1
            return

        elif state == 'Throw':
            next = self.throw_action()
            if next:
                self.current_state = 'Standby'
                # self.current_state = 'Pause'
                self.counter = 0
                self.stop_wheel()
                self.stop_thrower()
                return
            self.counter = self.counter + 1
            return

    # Callback from subscriptions
    def ball_callback(self, data):
        self.ball_x, self.ball_y, self.ball_d = data.x, data.y, data.d

    def basket_callback(self, data):
        self.basket_x, self.basket_y, self.basket_d = data.x, data.y, data.d

    def line_callback(self, data):
        self.line_x1, self.line_y1, self.line_x2, self.line_y2 = data.x1, data.y1, data.x2, data.y2

    def referee_callback(self, data):
        command_string = data.command
        if command_string == 'pause':
            self.current_state = 'Pause'

        elif command_string == 'resume':
            self.current_state = 'FindBall'
            self.execute_state(self.current_state)

    def pub_state_string(self):
        self.state_string.data = str(self.counter) + " : " + str(self.current_state)
        self.state_pub.publish(self.state_string)

    def above_line(self):
        if (self.line_x1 == -320 and self.line_y1 == 480) or (self.line_x2 == -320 and self.line_y2 == 480):
            return False
        else:
            return ((self.line_x2 - self.line_x1) * (self.ball_y - self.line_y1) - (self.line_y2 - self.line_y1) * (self.ball_x - self.line_x1)) > 0

    def rotation_orientation_with_line(self):
        if (self.line_x1 == -320 and self.line_y1 == 480) or (self.line_x2 == -320 and self.line_y2 == 480):
            return self.rot
        else:
            if self.line_y1 > self.line_y2:
                return -1   # CCW
            else:
                return 1    # CW

    """
     ___Actions to perform at each state___
     Return True to proceed to next state
     Return False to stay on same state
     Return False and self.current = new_state to change to specific state (Remember to reset counter)
    """

    def standby_action(self):
        return True

    def find_ball_action(self): # Finds ball and orients to ball.
        if self.detect_line:
            if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # No ball in sight
                moveValues = fball(self.rot)
                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                self.move.publish(self.msg)
                return False    # Continue rotation
            else:  # Ball in sight.
                if not self.above_line():   # Ball inside
                    # rospy.logwarn('Ball inside')
                    angle = calc_angle_cam(self.ball_x)
                    if abs(angle) > self.orientation_offset_find:
                        moveValues = orient(self.ball_x)
                        self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                            moveValues[2])
                        self.move.publish(self.msg)
                        return False  # Continue rotating until oriented to ball
                    return True  # Proceed to get_to_ball

                else:   # Ball outside
                    #rospy.logwarn('Ball outside')
                    # self.rot = self.rotation_orientation_with_line()  # Eventual logic
                    moveValues = fball(self.rot)
                    self.msg.w1, self.msg.w2, self.msg.w3 = 2*int(moveValues[0]), 2*int(moveValues[1]), 2*int(moveValues[2])
                    self.move.publish(self.msg)
                    return False  # Continue rotation


        else:   # Know this works
            if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # No ball in sight
                moveValues = fball(self.rot)
                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                self.move.publish(self.msg)
                return False    # Continue rotation
            else:  # Ball in sight.
                angle = calc_angle_cam(self.ball_x)
                if abs(angle) > self.orientation_offset_find:
                    moveValues = orient(self.ball_x)
                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                    self.move.publish(self.msg)
                    return False  # Continue rotating until oriented to ball
                return True     # Proceed to get_to_ball

    def get_to_ball_action(self):   # Moves towards ball until at a certain distance
        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:    # No ball in sight
            self.current_state = 'FindBall'
            self.counter = 0
            return False

        if self.detect_line:
            if self.above_line():
                self.current_state = 'FindBall'
                self.counter = 0
                # rospy.logwarn('DEBUG: Ball Outside court')
                return

        angle = calc_angle_cam(self.ball_x)
        if abs(angle) > self.orientation_offset_mov:
            self.current_state = 'FindBall'
            self.counter = 0
            # rospy.logwarn('DEBUG: Ball Unoriented')
            return

        else:   # Ball in sight
            angle = calc_angle(self.ball_x)
            # logangle = angle * 180 / np.pi
            # rospy.logwarn(logangle)
            yP, xP = tcc(self.ball_d, angle)
            if self.ball_d > self.min_ball_dist:  # Not yet near ball
                moveValues = approachBall(xP, yP)
                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                self.move.publish(self.msg)
                return False    # Continue until near
            else:   # Oriented and near
                return True # Proceed to im_at_ball

    def im_at_ball_action(self):    # Rotate around ball
        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # Ball lost
            self.current_state = 'FindBall'
            self.counter = 0
            # rospy.logwarn('DEBUG: Ball lost')
            return False

        ball_angle = calc_angle_cam(self.ball_x)

        if abs(ball_angle) > self.orientation_offset_rot:
            self.current_state = 'FindBall'
            self.counter = 0
            return False  # Continue rotating until oriented to ball

        if (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:
            moveValues = fbasket(1)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False    # Continue until basket is found

        basket_angle = calc_angle_cam(self.basket_x)

        if abs(basket_angle) > self.orientation_offset_pre_throw:
            if self.basket_x > 0:
                self.rot = 1
            else:
                self.rot = -1
            moveValues = fbasket(self.rot) # 1 or -1 according to rotation$
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False    # Continue rotating until oriented to basket
        else:   # Basket found
            return True


    def go_action(self):
        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # Ball lost
            self.current_state = 'FindBall'
            self.counter = 0
            # rospy.logwarn('DEBUG: Ball lost')
            return False

        if (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:    # Basket lost
            self.current_state = 'ImAtBall'
            self.counter = 0
            # rospy.logwarn('DEBUG: Basket lost')
            return False

        else:
            basket_angle = calc_angle_cam(self.basket_x)
            ball_angle = calc_angle_cam(self.ball_x)

            if abs(basket_angle) > self.orientation_offset_throw:   # Orientation to basket is off
                if self.basket_x > 0:
                    self.rot = 1
                else:
                    self.rot = -1
                moveValues = fbasket2(self.rot) # 1 or -1 according to rotation directions
                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                self.move.publish(self.msg)
                return False    # Continue rotating until oriented to basket
            if abs(ball_angle) > self.orientation_offset_throw_ball:
                if self.ball_x > 0:
                    self.rot = 1
                else:
                    self.rot = -1
                moveValues = fbasket2(self.rot) # 1 or -1 according to rotation directions
                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                self.move.publish(self.msg)
                return False    # Continue rotating until oriented to basket
            else:
                return True

    def go_action_alt(self):
        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # Ball lost
            self.current_state = 'FindBall'
            self.counter = 0
            return False

        if (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:    # Basket lost
            self.current_state = 'ImAtBall'
            self.counter = 0
            return False

        else:
            basket_angle = calc_angle(self.basket_x)
            ball_angle = calc_angle(self.ball_x)

            ball_xP, ball_yP = tcc(self.ball_d, ball_angle)
            basket_xP, basket_yP = tcc(self.basket_d, basket_angle)

            coord = align_throw(ball_xP, ball_yP, basket_xP, basket_yP, self.min_ball_dist)

            if coord[0] < 100 and coord[1] < 100:    # Ugh, dont' know what values .-.
                if basket_angle > self.orientation_offset_find:
                    moveValues = orient(self.basket_x)
                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                    self.move.publish(self.msg)
                    return False  # Continue rotating until oriented to basket
                else:
                    return True

            moveValues = approachBall(cood[0], coord[1])    # ApproachBall used as approach coordinate
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False  # Continue until near

    def throw_action(self):
        if (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:  # Basket lost
            self.current_state = 'FindBall'
            #self.current_state = 'Pause'
            self.counter = 0
            self.throwing_counter = 0
            self.thrower_msg.t1 = int(0)
            self.throw.publish(self.thrower_msg)
            # rospy.logwarn('DEBUG: Basket lost')
            return False

        elif self.throwing_counter >= self.rate * self.throw_duration:   # Termination of throwing
            self.throwing_counter = 0
            self.thrower_msg.t1 = int(0)
            self.throw.publish(self.thrower_msg)
            return True # Go find a new ball

        else:   # Turn on thrower and move forward
            throwerValue = thrower_calculation(self.basket_d)
            self.thrower_msg.t1 = int(throwerValue)
            self.throw.publish(self.thrower_msg)
            self.throwing_counter = self.throwing_counter + 1
            moveValues = -6, 6, 0     # Constant approach should result in constant throwing results.
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False    # Throwing

    def pause_action(self):
        pass

    def stuck_at_state_action(self):
        if (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:
            moveValues = fbasket(1)
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False    # Continue until basket is found

        basket_angle = calc_angle_cam(self.basket_x)

        if abs(basket_angle) > self.orientation_offset_stuck:
            if self.basket_x > 0:
                self.rot = 1
            else:
                self.rot = -1
            moveValues = fbasket(self.rot) # 1 or -1 according to rotation$
            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
            self.move.publish(self.msg)
            return False    # Continue rotating until oriented to basket

        if not ((self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0):
            if self.basket_d < 1500:
                moveValues = [8, -8, 0]
                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                self.move.publish(self.msg)
                return False
            elif self.basket_d > 1500:
                moveValues = [-8, 8, 0]
                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                self.move.publish(self.msg)
                return False

        else:
            return True

    def stuck_at_state_action_unused(self):
        if self.last_state == 'FindBall':
            # First half, same as findBall but slower
            if self.stuck_counter < self.stuck_max / 2:
                if self.detect_line:
                    if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # No ball in sight
                        moveValues = fball(self.rot * 0.5)
                        self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                        self.move.publish(self.msg)
                        return False  # Continue rotation
                    else:  # Ball in sight.
                        if not self.above_line():  # Ball inside
                            angle = calc_angle_cam(self.ball_x)
                            if abs(angle) > self.orientation_offset_find:
                                moveValues = orient(self.ball_x * 0.5)
                                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                                self.move.publish(self.msg)
                                return False  # Continue rotating until oriented to ball
                            return True  # Proceed to get_to_ball

                        else:  # Ball outside
                            self.rot = self.rotation_orientation_with_line()
                            moveValues = fball(self.rot * 0.5)
                            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                            self.move.publish(self.msg)
                            return False  # Continue rotation

                else:
                    if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # No ball in sight
                        moveValues = fball(self.rot)
                        self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                            moveValues[2])
                        self.move.publish(self.msg)
                        return False  # Continue rotation
                    else:  # Ball in sight.
                        angle = calc_angle_cam(self.ball_x)
                        if abs(angle) > self.orientation_offset_find:
                            moveValues = orient(self.ball_x)
                            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                                moveValues[2])
                            self.move.publish(self.msg)
                            return False  # Continue rotating until oriented to ball
                        return True  # Proceed to get_to_ball

            # Second half, move at certain distance to basket hopefully finding a ball
            elif self.stuck_counter < self.stuck_max:
                if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # No ball in sight
                    if (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:
                        moveValues = fbasket(1)
                        self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                            moveValues[2])
                        self.move.publish(self.msg)
                        return False  # Continue until basket is found

                    basket_angle = calc_angle_cam(self.basket_x)

                    if abs(basket_angle) > self.orientation_offset_find:  # Orientation to basket is off
                        if self.basket_x > 0:
                            self.rot = 1
                        else:
                            self.rot = -1
                        moveValues = fbasket(self.rot)  # 1 or -1 according to rotation directions
                        self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                            moveValues[2])
                        self.move.publish(self.msg)
                        return False  # Continue rotating until oriented to basket

                    elif self.basket_d > 2500:
                        moveValues = [-20, 20, 0]   # Forward
                        self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                        self.move.publish(self.msg)
                        return False

                    elif self.basket_d < 1500:
                        moveValues = [20, -20, 0]   # Backwards
                        self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                        self.move.publish(self.msg)
                        return False

                else:
                    if self.detect_line:
                        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # No ball in sight
                            moveValues = fball(self.rot * 0.5)
                            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                                moveValues[2])
                            self.move.publish(self.msg)
                            return False  # Continue rotation
                        else:  # Ball in sight.
                            if not self.above_line():  # Ball inside
                                angle = calc_angle_cam(self.ball_x)
                                if abs(angle) > self.orientation_offset_find:
                                    moveValues = orient(self.ball_x * 0.5)
                                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                                        moveValues[2])
                                    self.move.publish(self.msg)
                                    return False  # Continue rotating until oriented to ball
                                return True  # Proceed to get_to_ball

                            else:  # Ball outside
                                if self.line_x1 > 0:
                                    self.rot = 1
                                elif self.line_x1 < 0:
                                    self.rot = -1
                                moveValues = fball(self.rot * 0.5)
                                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                                    moveValues[2])
                                self.move.publish(self.msg)
                                return False  # Continue rotation

                    else:
                        if (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:  # No ball in sight
                            moveValues = fball(self.rot)
                            self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                                moveValues[2])
                            self.move.publish(self.msg)
                            return False  # Continue rotation
                        else:  # Ball in sight.
                            angle = calc_angle_cam(self.ball_x)
                            if abs(angle) > self.orientation_offset_find:
                                moveValues = orient(self.ball_x)
                                self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(
                                    moveValues[2])
                                self.move.publish(self.msg)
                                return False  # Continue rotating until oriented to ball
                            return True  # Proceed to get_to_ball

        elif self.last_state == 'GetToBall':    # Reset ball
            if not (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:
                if self.ball_d < self.min_ball_dist + 500: # Get away (most probably facing the basket way upclose)
                    moveValues = [20, -20, 0]  # Backwards
                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                    self.move.publish(self.msg)
                    return False

                else:
                    moveValues = fball(self.rot)
                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                    self.move.publish(self.msg)
                    return False  # Continue rotation
            else:
                return True

        elif self.last_state == 'ImAtBall':  # Reset Ball
            if not (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:
                if self.ball_d < self.min_ball_dist + 200:  # Get away (most probably facing the basket way upclose)
                    moveValues = [20, -20, 0]  # Backwards
                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                    self.move.publish(self.msg)
                    return False

                else:
                    moveValues = fball(self.rot)
                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                    self.move.publish(self.msg)
                    return False  # Continue rotation
            else:
                return True

        elif self.last_state == 'Go':  # Reset Ball
            if not (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:
                if self.ball_d < self.min_ball_dist + 500:
                    moveValues = [20, -20, 0]  # Backwards
                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                    self.move.publish(self.msg)
                    return False

                else:
                    moveValues = fball(self.rot)
                    self.msg.w1, self.msg.w2, self.msg.w3 = int(moveValues[0]), int(moveValues[1]), int(moveValues[2])
                    self.move.publish(self.msg)
                    return False
            else:
                return True

        elif self.last_state == 'Standby':
            return True

# ______________________________________________________________________________________________________________________

# No ball = (self.ball_x == -320 and self.ball_y == 480) or self.ball_d == 0:
# No basket = (self.basket_x == -320 and self.basket_y == 480) or self.basket_d == 0:

# rostopic pub /referee lebot/Ref_Command "{'command':'resume'}"

if __name__ == '__main__':
    rospy.init_node('state_machine')
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    fb = Logic(min_dist=300, node_rate=myRate, line_detection=True, stuck_activated=False)
    fb.current_state = 'Pause'
    while not rospy.is_shutdown():
        fb.execute_state(fb.current_state)
        rate.sleep()
