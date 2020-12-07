#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Report
from lebot.msg import Wheel
from lebot.msg import Thrower


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
        self.thrower_speed = 50

    def controller_callback(self, data):
        report = Report()
        report = data

        self.message.w1 = 0
        self.message.w2 = 0
        self.message.w3 = 0
        self.thrower_message.t1 = 0

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

        self.controller_pub.publish(self.message)

        if report.button_cross:
            self.thrower_message.t1 = self.thrower_speed

        self.thrower_pub.publish(self.thrower_message)

if __name__ == '__main__':
    rospy.init_node('controller_input', anonymous=False)
    theC = Cont()
    rospy.spin()

