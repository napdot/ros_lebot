#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Report
from lebot.msg import Wheel
from lebot.msg import Thrower
from scipy.interpolate import interp1d
import numpy as np


class Cont:
    def __init__(self):
        self.wheelDistanceFromCenter = 10
        self.wheelAngle = [0, 120, 240]
        self.robotAngularVelocity = 10
        self.default_speed = 70
        self.controller_sub = rospy.Subscriber("/raw_report", Report, self.controller_callback, queue_size=10)
        self.controller_pub = rospy.Publisher('/wheel_values', Wheel, queue_size=10)
        self.message = Wheel()
        self.thrower_pub = rospy.Publisher('/thrower_values', Thrower, queue_size=10)
        self.thrower_message = Thrower()
        self.thrower_speed = 1800
        self.maxSpeedEnc = 190

    def controller_callback(self, data):
        report = Report()
        report = data

        self.message.w1 = 0
        self.message.w2 = 0
        self.message.w3 = 0
        self.thrower_message.t1 = 1000

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

        elif (133 > report.left_analog_x > 123) or (133 > report.left_analog_y > 123):
            self.message.w1, self.message.w2, self.message.w3 = self.joy_to_omni(report.left_analog_x, report.left_analog_x)

        self.controller_pub.publish(self.message)

        if report.button_cross:
            self.thrower_message.t1 = self.thrower_speed

        self.thrower_pub.publish(self.thrower_message)

    def joy_to_omni(self, x, y):
        m = interp1d([-self.maxSpeedEnc, self.maxSpeedEnc], [0, 255])
        x_val = m(x)
        y_val = m(y)
        aKI = np.array([[np.sqrt(3) / 3, 1 / 3, 1 / 3], [-np.sqrt(3) / 3, 1 / 3, 1 / 3], [0, -2 / 3, 1 / 3]])
        m = np.dot(aKI, np.array([x_val, y_val, np.arctan2(x_val, y_val)]))
        mSer = np.rint(np.multiply(np.multiply(np.divide(m, np.max(np.absolute(m))), 190), 0.2))
        return int(mSer[0]), int(mSer[1]), int(mSer[2])



if __name__ == '__main__':
    rospy.init_node('controller_input', anonymous=False)
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    theC = Cont()
    rate.sleep()
    rospy.spin()

