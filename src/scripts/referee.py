#!/usr/bin/env python3

import rospy
from rospy_websocker_client import WebsocketROSClient as ros_ws
from st_msgs.msg import String
from lebot.msg import Ref_Command


# https://github.com/GigaFlopsis/rospy_websocker_client

class Signal:
    def __init__(self):
        self.ws_client.Subscriber('/ref_signal', String(), signal_callback())
        self.ref_signals.Publisher('/referee', Ref_Command()

    def signal_callback(self, data):
        self.ws



if __name__ == '__main__':
    rospy.init_node('ref', anonymous=False)
    Signal()
    rospy.spin()


