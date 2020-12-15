#!/usr/bin/env python3

import rospy
from st_msgs.msg import String
from lebot.msg import Ref_Command

class Signal:
    def __init__(self, topic):
        self.sub_topic = topic
        self.ws_client.Subscriber(self.sub_topic, String(), self.signal_callback())
        self.ref_signals.Publisher('/referee', Ref_Command())

    def signal_callback(self, str):
        string = str.data
        st




if __name__ == '__main__':
    rospy.init_node('ref', anonymous=False)
    topic = '/js_ref'
    Signal(topic)
    rospy.spin()


