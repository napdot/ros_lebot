#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from lebot.msg import Depth_BallLocation
import cv2
import json


class Ball:
    def __init__(self):
        self.green_parameters = []
        self.ball_location = [0, 0]
        self.ball_distance = 0
        self.set_ball_parameters()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_my_depth_callback, queue_size=1)
        self.ball_depth_location_pub = rospy.Publisher("ball", Depth_BallLocation, queue_size=1)
        self.hsv = np.zeros((480, 650, 3), np.uint16)
        self.mask = np.zeros((480, 650, 3), np.uint16)
        self.depth = np.zeros((480, 650, 3), np.uint16)
        self.ball_message = Depth_BallLocation()
        self.kernel = np.ones((3, 3), np.uint8)
        self.depth_bridge = CvBridge()
        self.color_bridge = CvBridge()

    def get_my_image_callback(self, data):
        try:
            color_image = self.color_bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
            color_image = np.zeros((480, 650, 3), np.uint16)

        self.hsv = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)
        self.get_mask()

    def get_my_depth_callback(self, data):
        try:
            depth_image = self.depth_bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
            depth_image = np.zeros((480, 650, 3), np.uint16)

        self.depth = depth_image
        self.get_ball_location()
        self.get_depth_to_ball()
        self.update_ball_message()
        self.ball_depth_location_pub.publish(self.ball_message)

    def get_mask(self):
        self.mask = cv2.inRange(self.hsv, tuple(self.green_parameters['min']), tuple(self.green_parameters['max']))
        self.mask = (cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, self.kernel)).astype(np.uint8)

    def get_depth_to_ball(self):
        selected_depth_array = self.depth * self.mask
        self.ball_distance = np.mean(selected_depth_array)

    def update_ball_message(self):
        self.ball_message.x = self.ball_location[0]
        self.ball_message.y = self.ball_location[1]
        self.ball_message.d = self.ball_distance

    def get_ball_location(self):
        radius_min = 10
        radius_max = 50

        cnts = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), r) = cv2.minEnclosingCircle(c)
            if (r < radius_max) & (r > radius_min):
                cX = x
                cY = y
            else:
                cX = 0
                cY = 0
        else:
            cX = 0
            cY = 0

        self.ball_location = [cX, cY]

    def set_ball_parameters(self):
        try:
            with open('color_parameters.json') as f:
                d = json.load(f)
                self.green_parameters = d['green']
        except:
            self.green_parameters = {"min": [51, 131, 0], "max": [81, 226, 255]}


if __name__ == '__main__':
    rospy.init_node('ball_calc', anonymous=False)
    IWhoFindsABall = Ball()
    rospy.spin()

