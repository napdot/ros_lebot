import rospy
from msg.msg import Wheel
import serial


class Move:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", timeout=0.03, baudrate=115200,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

        self.wheel_sub = rospy.Subscriber("/wheel_values", Wheel, self.wheel_callback)

    def move_to(self, w1, w2, w3):
        sot = ("sd:{0}:{1}:{2}\n".format(w1, w2, w3))
        self.ser.write(sot.encode('utf-8'))

    def wheel_callback(self, data):
        self.move_to(data.w1, data.w2, data.w3)


if __name__ == '__main__':
    rospy.init_node('/vruum', anonymous=False)
    Move()
    rospy.spin()
