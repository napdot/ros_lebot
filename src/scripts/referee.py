#!/usr/bin/env python3
import rospy
from rospy_websocker_client import WebsocketROSClient as ros_ws
from st_msgs.msg import String

# https://github.com/GigaFlopsis/rospy_websocker_client

class Signal:
    def __init__(self):
        self.ws_client.Subscriber('/ref_signal', String(), signal_callback())
        self.ref_signals.Publisher('/referee', String(), signal_callback())

    def signal_callback(self, data):





def main():
    rospy.init_node('/ref')
    Signal()
    rospy.spin()


if __name__ == '__main__':
    main()




