import rospy
from additional_msgs.msg import Wheel
import serial


class Move:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", timeout=0.03, baudrate=115200,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

    def move_to(self, w1, w2, w3):
        sot = ("sd:{0}:{1}:{2}\n".format(w1, w2, w3))
        self.ser.write(sot.encode('utf-8'))


def wheel_callback(data):
    global w1, w2, w3
    w1 = data.w1
    w2 = data.w2
    w3 = data.w3
    meMove.move_to(w1, w2, w3)


if __name__ == '__main__':
    global w1, w2, w3
    meMove = Move
    rospy.init_node('/vruum', anonymous=False)
    wheel_sub = rospy.Subscriber("/wheel_values", Wheel, wheel_callback)
    rospy.spin()

