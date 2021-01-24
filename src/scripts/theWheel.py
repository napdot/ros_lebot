#!/usr/bin/env python3
import rospy
from lebot.msg import Wheel
import serial
from lebot.msg import Thrower


class Move:
    def __init__(self, nelli):
        self.ser = serial.Serial("/dev/ttyACM0", timeout=0.03, baudrate=115200,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

        self.wheel_sub = rospy.Subscriber("/wheel_values", Wheel, self.wheel_callback, queue_size=1)
        self.thrower_sub = rospy.Subscriber("/thrower_values", Thrower, self.thrower_callback, queue_size=1)
        self.ser_type = nelli
        self.correction = [-1, -1, -1]

    def move_to(self, w1, w2, w3):
        if self.ser_type:
            sot = ("sd:{0}:{1}:{2}\n".format(w1, w2, w3))
        else:
            sot = ("s<{0}:{1}:{2}>\n".format(w1, w2, w3))
        self.ser.write(sot.encode('utf-8'))
        rospy.loginfo(sot)

    def wheel_callback(self, data):
        w1 = int(data.w1 * self.correction[0])
        w2 = int(data.w2 * self.correction[1])
        w3 = int(data.w3 * self.correction[2])
        self.move_to(w1, w2, w3)

    def throw_at(self, t1):
        if self.ser_type:
            tot = ("d:{0}\n".format(t1))
        else:
            tot = ("t<{0}\n>".format(t1))
        self.ser.write(tot.encode('utf-8'))
        rospy.loginfo(tot)

    def thrower_callback(self, data):
        self.throw_at(data.t1)


if __name__ == '__main__':
    rospy.init_node('vruum', anonymous=False)
    # True is test robot. False for LeBot.
    IWhoMoves = Move(nelli=False)
    rospy.spin()
