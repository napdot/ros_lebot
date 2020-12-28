#!/usr/bin/env python3
import rospy
from lebot.msg import Wheel
import serial
from lebot.msg import Thrower


class Move:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", timeout=0.03, baudrate=115200,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

        self.wheel_sub = rospy.Subscriber("/wheel_values", Wheel, self.wheel_callback, queue_size=1)
        self.thrower_sub = rospy.Subscriber("/thrower_values", Thrower, self.thrower_callback, queue_size=1)

    def move_to(self, w1, w2, w3):
        sot = ("sd:{0}:{1}:{2}\n".format(w1, w2, w3))
        self.ser.write(sot.encode('utf-8'))
        rospy.loginfo(sot)

    def wheel_callback(self, data):
        self.move_to(data.w1, data.w2, data.w3)

    def throw_at(self, t1):
        tot = ("d:{0}\n".format(t1))
        self.ser.write(tot.encode('utf-8'))
        rospy.loginfo(tot)

    def thrower_callback(self, data):
        self.throw_at(data.t1)


if __name__ == '__main__':
    rospy.init_node('vruum', anonymous=False)
    IWhoMoves = Move()
    rospy.spin()
