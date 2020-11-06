#!/usr/bin/env python

import rospy
from ds4_driver.msg.msg import Report
from additional_msgs.msg import Wheel

class Cont:
    def __init__(self):
        self.message = Wheel
        self.default_speed = 10
        controller_sub = rospy.Suscriber("/raw_report", Report, queue_size=1)
        controller_pub = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        controller_pub.publush(self.message)

    def controller_callback(self, data):
        report = Report()
        report = data
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



if __name__ == '__main__':
    rospy.init_node('/controller_input', anonymous=False)
    Cont()
    rospy.spin()
