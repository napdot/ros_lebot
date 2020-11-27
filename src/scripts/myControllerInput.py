#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Report
from msg.msg import Wheel
from math import atan2, cos, sqrt


class Cont:
    def __init__(self):
        self.wheelDistanceFromCenter = 10
        self.wheelAngle = [0, 120, 240]
        self.robotAngularVelocity = 10
        self.message = Wheel
        self.default_speed = 10
        self.controller_sub = rospy.Suscriber("/raw_report", Report, self.controller_callback, queue_size=1)
        self.controller_pub = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.controller_pub.publush(self.message)

    def controller_callback(self, data):
        report = Report()
        report = data

        if report.left_analog_x or report.left_analog_x:
            self.message.w1, self.message.w2, self.message.w3 = \
                self.omni(report.left_analog_x, report.left_analog_y)
        else:
            if report.dpad_up:
                self.message.w1 = self.default_speed
                self.message.w2 = -self.default_speed
                self.message.w2 = 0

            elif report.dpad_down:
                self.message.w1 = -self.default_speed
                self.message.w2 = self.default_speed
                self.message.w2 = 0

            elif report.dpad_left:
                self.message.w1 = -self.default_speed
                self.message.w2 = -self.default_speed
                self.message.w2 = -self.default_speed

            elif report.dpad.right:
                self.message.w1 = self.default_speed
                self.message.w2 = self.default_speed
                self.message.w2 = self.default_speed

    def omni(self, speed_x, speed_y):
        x = int(speed_x / 250)
        y = int(speed_y / 250)
        try:
            robotDirectionAngle = atan2(x, y)
        except:
            robotDirectionAngle = 0.01

        robotSpeed = sqrt(x * x + y * y)

        wheelLinearVelocity = [0, 0, 0]
        for i in range(3):
            wheelLinearVelocity[i] = robotSpeed * cos(robotDirectionAngle - self.wheelAngle[i]) \
                                     + self.wheelDistanceFromCenter * self.robotAngularVelocity

        return wheelLinearVelocity[0], wheelLinearVelocity[1], wheelLinearVelocity[2]


if __name__ == '__main__':
    rospy.init_node('/controller_input', anonymous=False)
    Cont()
    rospy.spin()
