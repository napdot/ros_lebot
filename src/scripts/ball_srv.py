#!/usr/bin/env python3
import rospy
from lebot.msg import Depth_BallLocation
from lebot.srv import ball

class BallService:
    def __init__(self):
        self.ball_msg = Depth_BallLocation()
        self.ball_serv = rospy.Service('/ball_service', ball, self.ball_service_callback)
        self.ball_sub = rospy.Subscriber('/ball', Depth_BallLocation, self.ball_sub_callback, queue_size=10)

    def ball_service_callback(self, data):
        return ball(
            x=self.ball_msg.x,
            y=self.ball_msg.y,
            d=self.ball_msg.d
        )

    def ball_sub_callback(self, data):
        self.ball_msg.x = data.x
        self.ball_msg.y = data.y
        self.ball_msg.d = data.d


if __name__ == "__main__":
    rospy.init_node('ball_node_srv')
    theBallService = BallService()
    rospy.spin()