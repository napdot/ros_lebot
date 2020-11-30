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
<<<<<<< HEAD
        self.message = Wheel()
=======
>>>>>>> 92325e868eb0245370188b11bcca8f64d5233c6c

    def controller_callback(self, data):
        report = Report()
        report = data

<<<<<<< HEAD
        self.message.w1 = 0
        self.message.w2 = 0
        self.message.w3 = 0

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
=======
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

        elif report.dpad_right:
            message.w1 = self.default_speed
            message.w2 = self.default_speed
            message.w3 = self.default_speed

        self.controller_pub.publish(message)
>>>>>>> 92325e868eb0245370188b11bcca8f64d5233c6c


if __name__ == '__main__':
    rospy.init_node('controller_input', anonymous=False)
    theC = Cont()
    rospy.spin()

