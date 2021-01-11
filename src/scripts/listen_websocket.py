#!/usr/bin/python3

import rospy
import websockets
import ssl


if __name__ == '__main__':
    rospy.init_node('listen_ws', anonymous=False)
    address = "ws://localhost:8080/websocket"
    webs = rospy.Publisher('/js_ref', Ref_Command, queue_size=10)
    with websockets.connect(address) as ws:
        while not rospy.is_shutdown:
            response = ws.recv()
            webs.publish(response)
            rospy.spin()
