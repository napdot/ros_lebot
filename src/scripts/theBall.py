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
        self.ball_radius = 0
        self.set_ball_parameters()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_my_depth_callback)
        self.ball_depth_location_pub = rospy.Publisher("ball", Depth_BallLocation, queue_size=10)
        self.hsv = np.zeros((480, 650, 3), np.uint16)
        self.thresh = np.zeros((480, 650, 3), np.uint16)
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
        self.get_thresh()
        return

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
        return

    def get_thresh(self):
        self.thresh = cv2.inRange(self.hsv, tuple(self.green_parameters['min']), tuple(self.green_parameters['max']))
        return

    def get_depth_to_ball(self):
        try:
            self.ball_distance = np.mean(self.depth * self.thresh)
            #thresh_depth_array = self.depth * self.thresh
            #self.ball_distance = np.mean(thresh_depth_array[x1:x2, y1:y2])
        except:
            self.ball_distance = 0

        if self.ball_location[0] == 0 or self.ball_location[1] == 0:
            self.ball_distance = 0
        return

    def update_ball_message(self):
        self.ball_message.x = int(self.ball_location[0])
        self.ball_message.y = int(self.ball_location[1])
        self.ball_message.d = int(self.ball_distance)
        return

    def get_ball_location(self):
        radius_min = 10
        radius_max = 300
        self.ball_location = [0, 0]
        try:
            im, cnts, hier = cv2.findContours(self.thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(cnts) != 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), r) = cv2.minEnclosingCircle(c)
                if (r < radius_max) and (r > radius_min):
                    cX = x
                    cY = y
                    cR = r
                else:
                    cX = 0
                    cY = 0
                    cR = 0
            else:
                cX = 0
                cY = 0
                cR = 0
        except:
            cX = 0
            cY = 0
            cR = 0

        self.ball_location = [cX, cY]
        self.ball_radius = cR
        return

    def get_area_of_interest(self):
        x1 = self.ball_location[0] - self.ball_radius
        x2 = self.ball_location[0] - self.ball_radius
        y1 = self.ball_location[1] - self.ball_radius
        y2 = self.ball_location[0] - self.ball_radius
        return int(x1), int(x2), int(y1), int(y2)

    def set_ball_parameters(self):
        try:
            with open('color_parameters.json') as f:
                d = json.load(f)
                self.green_parameters = d['green']
        except:
            self.green_parameters = {"min": [51, 131, 0], "max": [81, 226, 255]}
        return


if __name__ == '__main__':
    rospy.init_node('ball_calc', anonymous=False)
    Ball()
    rospy.spin()

