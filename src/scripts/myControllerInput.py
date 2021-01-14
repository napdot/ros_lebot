#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Report
from std_msgs.msg import String
from lebot.msg import Wheel
from lebot.msg import Thrower
from scipy.interpolate import interp1d
import numpy as np
#from omni import omni_to_serial as ots
from movement.approachBall import approachBall as ots


def dz_radial(x_inp, y_inp, deadzone):
    input_magnitude = np.linalg.norm([x_inp, y_inp])
    if input_magnitude < deadzone:
        return 0, 0
    else:
        return x_inp, y_inp

def interp_input(x_inp, y_inp):
    x_norm = np.interp(x_inp, [0, 255], [-320, 320])
    y_norm = np.interp(y_inp, [0, 255], [-480, 480])
    return x_norm, y_norm


class Cont:
    def __init__(self):
        self.wheelDistanceFromCenter = 10
        self.wheelAngle = [0, 120, 240]
        self.robotAngularVelocity = 10
        self.default_speed = 20
        self.controller_max_speed = 20
        self.controller_sub = rospy.Subscriber("/raw_report", Report, self.controller_callback, queue_size=1)
        self.controller_pub = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.game_logic_pub = rospy.Publisher('/js_ref', String, queue_size=1)
        self.message = Wheel()
        self.thrower_pub = rospy.Publisher('/thrower_values', Thrower, queue_size=1)
        self.thrower_message = Thrower()
        self.thrower_speed = 1000
        self.thrower_min = 1000
        self.thrower_max = 2000
        self.maxSpeedEnc = 100
        self.was_throwing = False

    def controller_callback(self, data):
        report = Report()
        report = data

        self.message.w1 = 0
        self.message.w2 = 0
        self.message.w3 = 0
        self.thrower_message.t1 = 1000

        if report.button_square:
            self.thrower_speed = self.thrower_speed + 1
            if self.thrower_speed >= self.thrower_max:
                self.thrower_speed = self.thrower_max

        elif report.button_circle:
            self.thrower_speed = self.thrower_speed - 1
            if self.thrower_speed <= self.thrower_min:
                self.thrower_speed = self.thrower_min

        if report.dpad_up:
            self.message.w1 = -self.default_speed
            self.message.w2 = self.default_speed
            self.message.w3 = 0
            self.controller_pub.publish(self.message)

        elif report.dpad_down:
            self.message.w1 = self.default_speed
            self.message.w2 = -self.default_speed
            self.message.w3 = 0
            self.controller_pub.publish(self.message)

        elif report.dpad_left:
            self.message.w1 = -self.default_speed
            self.message.w2 = -self.default_speed
            self.message.w3 = -self.default_speed
            self.controller_pub.publish(self.message)

        elif report.dpad_right:
            self.message.w1 = self.default_speed
            self.message.w2 = self.default_speed
            self.message.w3 = self.default_speed
            self.controller_pub.publish(self.message)

        elif report.left_analog_x or report.left_analog_y:
            x, y = interp_input(report.left_analog_x, report.left_analog_y)
            new_x, new_y = dz_radial(x, y, 40)
            if new_x != 0 or new_y != 0:
                moveValues = ots(new_y, new_x)
                w1, w2, w3 = moveValues
                self.message.w1, self.message.w2, self.message.w3 = int(w1), int(w2), int(w3)
                self.controller_pub.publish(self.message)

        if report.button_options:
            start_msg = String()
            start_msg.data = '{"signal":"start","targets":["LeBot"],"baskets":["blue"]}'
            self.game_logic_pub.publish(start_msg)

        if report.button_share:
            stop_msg = String()
            stop_msg.data = '{"signal":"stop","targets":["LeBot"]}'
            self.game_logic_pub.publish(stop_msg)

        if report.button_cross:
            self.thrower_message.t1 = self.thrower_speed
            self.thrower_pub.publish(self.thrower_message)
            self.was_throwing = True

        elif (not report.button_cross) and self.was_throwing:
            self.thrower_message.t1 = 1000
            self.thrower_pub.publish(self.thrower_message)
            self.was_throwing = False


if __name__ == '__main__':
    rospy.init_node('controller_input', anonymous=False)
    theC = Cont()
    rospy.spin()

