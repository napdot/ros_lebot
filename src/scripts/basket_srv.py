#!/usr/bin/env python3
import rospy
from lebot.msg import Depth_BasketLocation
from lebot.srv import basket


class BasketService:
    def __init__(self):
        self.basket_msg = Depth_BasketLocation
        self.basket_sub = rospy.Subscriber('/basket', Depth_BasketLocation, self.basket_sub_callback, queuesize=1)
        self.ball_serv = rospy.Service('/basket_service', self.basket_service_callback)

    def basket_service_callback(self):
        return basket(
            x=self.basket_msg.x,
            y=self.basket_msg.y,
            d=self.basket_msg.d
        )

    def basket_sub_callback(self, data):
        self.basket_msg.x = data.x
        self.basket_msg.y = data.y
        self.basket_msg.d = data.d


if __name__ == "__main__":
    rospy.init_node('/basket_node_srv')
    BasketService()
    rospy.spin()
