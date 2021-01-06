#!/usr/bin/env python3

import os
dir_path = os.path.dirname(os.path.realpath(__file__))

import rospy
from std_msgs.msg import String
from lebot.msg import Ref_Command
import json
import webbrowser

class Signal:
    def __init__(self):
        self.ws_client = rospy.Subscriber('/js_ref', String, self.signal_callback, queue_size=10)
        self.ref_signals = rospy.Publisher('/referee', Ref_Command, queue_size=10)
        self.color_pub = rospy.Publisher('/color_ref', String, queue_size=1)

        # !!! This path is so ugly.
        webbrowser.open_new('../test_lebot/src/ros_lebot/src/scripts/ref2.html')

    def signal_callback(self, string):
        ref_string = string.data
        ref_obj = json.loads(ref_string)
        targets = ref_obj['targets']
        try:    # if target.index returns error, means that we are not on target list and don't need to change anything.
            index = targets.index("LeBot")
            cm = Ref_Command()
            if ref_obj['signal'] == 'start':
                cm.command = 'resume'
                self.ref_signals.publish(cm)
                color = ref_obj['targets'][index]
                self.color_pub.publish(color)
                rospy.set_param("basket_color", color)
            elif ref_obj['signal'] == 'stop':
                cm.command = 'pause'
                self.ref_signals.publish(cm)
        except:
            rospy.logwarn("Referee signal callback not targeting LeBot (Check capitalization)")
            pass

if __name__ == '__main__':
    rospy.init_node('ref', anonymous=False)
    Signal()
    rospy.spin()
