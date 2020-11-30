#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from lebot.msg import Depth_BasketLocation
import cv2
import json


class Basket:
    def __init__(self, color):
        self.red_parameters = None
        self.blue_parameters = None
        self.basket_location = [0, 0]
        self.basket_distance = 0
        self.basket_edges = [0, 0, 0, 0]
        self.set_basket_parameters()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_my_depth_callback)
        self.basket_depth_location_pub = rospy.Publisher("basket", Depth_BasketLocation, queue_size=1)
        self.mask = None
        self.color = color
        self.basket_message = Depth_BasketLocation()

    def get_my_image_callback(self, data):
        bridge_img = CvBridge()
        image = bridge_img.compressed_imgmsg_to_cv2(data, "bgr8")
        self.get_basket_location(image)

    def get_my_depth_callback(self, data):
        bridge_depth = CvBridge()
        depth_image = bridge_depth.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.depth_to_basket(depth_image)
        self.basket_message.x = 0
        self.basket_message.y = 0
        self.basket_message.d = 0
        self.update_basket_message()
        self.basket_depth_location_pub.publish(self.basket_message)

    def depth_to_basket(self, depth_img):
        selected_depth_array = depth_img * self.mask
        # selected_depth_array = np.mean(depth_img[self.basket_edges[3]:self.basket_edges[2], self.basket_edges[1]: self.basket_edges[0]])
        self.basket_distance = np.mean(selected_depth_array)

    def update_basket_message(self):
        self.basket_message.x = self.basket_location[0]
        self.basket_message.y = self.basket_location[1]
        self.basket_message.d = self.basket_distance

    def get_basket_location(self, frame, color):

        if color == 'red':
            self.mask = cv2.inRange(frame, tuple(self.red_parameters['min']), tuple(self.red_parameters['max']))
        elif color == 'blue':
            self.mask = cv2.inRange(frame, tuple(self.blue_parameters['min']), tuple(self.blue_parameters['max']))

        basket_contours, hierarchy = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        x1, y1, w, h = cv2.boundingRect(basket_contours)
        x2, y2 = x1 + w, y1 + h
        self.basket_edges = [x1, x2, y1, y2]
        M = cv2.moments(basket_contours)
        self.basket_location = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    def set_basket_parameters(self):
        try:
            with open('color_parameters.json') as f:
                d = json.load(f)
                self.red_parameters = d['red']
                self.blue_parameters = d['blue']

        except:
            self.red_parameters = {"min": [151, 98, 87], "max": [179, 232, 214]}
            self.blue_parameters ={"min": [99, 119, 64], "max": [118, 255, 175]}


if __name__ == '__main__':
    rospy.init_node('basket_calc', anonymous=False)
    IWhoFindsABasket = Basket('red')
    rospy.spin()
