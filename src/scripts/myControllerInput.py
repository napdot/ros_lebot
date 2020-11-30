#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Report
from lebot.msg import Wheel


class Cont:
    def __init__(self):
        self.wheelDistanceFromCenter = 10
        self.wheelAngle = [0, 120, 240]
        self.robotAngularVelocity = 10
        self.default_speed = 10
        self.controller_sub = rospy.Subscriber("/raw_report", Report, self.controller_callback, queue_size=10)
        self.controller_pub = rospy.Publisher('/wheel_values', Wheel, queue_size=10)

    def controller_callback(self, data):
        report = Report()
        report = data

        message = Wheel
        message.w1, message.w2, message.w3 = 0, 0, 0

        if report.dpad_up:
            message.w1 = self.default_speed
            message.w2 = -self.default_speed
            message.w3 = 0

        elif report.dpad_down:
            message.w1 = -self.default_speed
            message.w2 = self.default_speed
            message.w3 = 0

        elif report.dpad_left:
            message.w1 = -self.default_speed
            message.w2 = -self.default_speed
            message.w3 = -self.default_speed

        elif report.dpad.right:
            message.w1 = self.default_speed
            message.w2 = self.default_speed
            message.w3 = self.default_speed

        self.controller_pub.publish(message)


if __name__ == '__main__':
    rospy.init_node('controller_input', anonymous=False)
    theC = Cont()
    rospy.spin()


