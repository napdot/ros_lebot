#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Report
from lebot.msg import Wheel
from lebot.msg import Thrower
from scipy.interpolate import interp1d
import numpy as np
from omni import omni_to_serial as ots


class Cont:
    def __init__(self):
        self.wheelDistanceFromCenter = 10
        self.wheelAngle = [0, 120, 240]
        self.robotAngularVelocity = 10
        self.default_speed = 70
        self.controller_sub = rospy.Subscriber("/raw_report", Report, self.controller_callback, queue_size=1)
        self.controller_pub = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.message = Wheel()
        self.thrower_pub = rospy.Publisher('/thrower_values', Thrower, queue_size=1)
        self.thrower_message = Thrower()
        self.thrower_speed = 1000
        self.maxSpeedEnc = 190

    def controller_callback(self, data):
        report = Report()
        report = data

        self.message.w1 = 0
        self.message.w2 = 0
        self.message.w3 = 0
        self.thrower_message.t1 = 1000
	
        if report.button_square:
            self.thrower_speed = self.thrower_speed + 1

        elif report.button_circle:
            self.thrower_speed = self.thrower_speed - 1

        if report.dpad_up:
            self.message.w1 = self.default_speed
            self.message.w2 = -self.default_speed
            self.message.w3 = 0

        elif report.dpad_down:
            self.message.w1 = -self.default_speed
            self.message.w2 = self.default_speed
            self.message.w3 = 0

        elif report.dpad_left:
            self.message.w1 = -self.default_speed
            self.message.w2 = -self.default_speed
            self.message.w3 = -self.default_speed

        elif report.dpad_right:
            self.message.w1 = self.default_speed
            self.message.w2 = self.default_speed
            self.message.w3 = self.default_speed

        if not ((138 > report.left_analog_x > 118) and (138 > report.left_analog_y > 118)):
            y_interp = np.interp(report.left_analog_y, [0, 255], [-self.default_speed, self.default_speed])
            x_interp = np.interp(report.left_analog_x, [0, 255], [-self.default_speed, self.default_speed])
            self.message.w1, self.message.w2, self.message.w3 = ots(y_interp, x_interp)

        self.controller_pub.publish(self.message)

        if report.button_cross:
            self.thrower_message.t1 = self.thrower_speed

        self.thrower_pub.publish(self.thrower_message)



if __name__ == '__main__':
    rospy.init_node('controller_input', anonymous=False)
    theC = Cont()
    rospy.spin()

