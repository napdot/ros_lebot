#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from lebot.msg import Depth_BasketLocation
import cv2
import json
import os


class Basket:
    def __init__(self, color, alt_dist):
        self.color = color
        self.red_parameters = []
        self.blue_parameters = []
        self.basket_location = [0, 0]
        self.basket_distance = 0

        self.alt_dist = alt_dist

        self.set_basket_parameters()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_my_depth_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("/color_ref", String, self.color_callback, queue_size=1)

        self.basket_depth_location_pub = rospy.Publisher("basket", Depth_BasketLocation, queue_size=1)
        self.hsv = np.zeros((480, 650, 3), np.uint16)
        self.thresh = np.zeros((480, 650, 3), np.uint16)
        self.depth = np.zeros((480, 650, 3), np.uint16)
        self.basket_message = Depth_BasketLocation()
        self.kernel = np.ones((3, 3), np.uint8)
        self.depth_bridge = CvBridge()
        self.color_bridge = CvBridge()

    def color_callback(self, data):
        self.color = data.data
        rospy.logwarn(str(self.color))

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
            self.depth = np.array(self.depth_bridge.imgmsg_to_cv2(data, desired_encoding="passthrough"), dtype=np.float32)
        except CvBridgeError as e:
            print(e)
            self.depth = np.zeros((480, 650), np.float32)

        self.get_basket_location()
        self.get_depth_to_basket()
        self.update_basket_message()
        self.basket_depth_location_pub.publish(self.basket_message)
        return

    def get_thresh(self):
        kernel = np.ones((3, 3), np.uint8)
        if self.color == 'red':
            thresh = cv2.inRange(self.hsv, tuple(self.red_parameters['min']), tuple(self.red_parameters['max']))
        elif self.color == 'blue':
            thresh = cv2.inRange(self.hsv, tuple(self.blue_parameters['min']), tuple(self.blue_parameters['max']))

        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        self.thresh = cv2.dilate(thresh, kernel, iterations=1)
        return

    def get_depth_to_basket(self):
        self.basket_distance = 0
        try:
            self.basket_distance = np.mean(self.depth * self.thresh)
        except:
            self.basket_distance = 0

        if self.alt_dist:
            try:
                self.basket_distance = self.depth[int(self.basket_location[1]), int(self.basket_location[0])]
            except:
                self.basket_distance = 0

        if self.basket_location[0] == 0 or self.basket_location[1] == 0:
            self.basket_distance = 0
        return

    def update_basket_message(self):
        self.basket_message.x, self.basket_message.y = self.transform_location(self.basket_location)
        self.basket_message.d = int(self.basket_distance)
        return

    def get_basket_location(self):
        min_basket_area = 900
        try:
            cnt = cv2.findContours(self.thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            areas = []
            max_index = -1
            max_area = 0
            for index, c in enumerate(cnt):
                area_of_cnt = cv2.contourArea(c)
                areas.append(area_of_cnt)
                if area_of_cnt > max_area:
                    max_area = area_of_cnt
                    max_index = index
            contour = cnt[max_index]
            if (cv2.contourArea(contour) > min_basket_area) and (len(cnt) > 0):
                M = cv2.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                self.basket_location = [cX, cY]
            else:
                self.basket_location = [0, 0]

        except:
            self.basket_location = [0, 0]


    def set_basket_parameters(self):
        try:
            dir = '../test_lebot/src/ros_lebot/src/scripts/color_parameters.json'
            with open(dir) as f:
                d = json.load(f)
                self.red_parameters = d['red']
                self.blue_parameters = d['blue']
                rospy.loginfo('Basket masking parameters set')

        except:
            self.red_parameters = {"min": [151, 98, 87], "max": [179, 232, 214]}
            self.blue_parameters ={"min": [99, 119, 64], "max": [118, 255, 175]}
            rospy.logwarn("Error reading json - Using defaults (Basket)")

    def transform_location(self, loc):
        tx = int(np.interp((loc[0]), [0, 640], [-320, 320]))
        ty = int(480 - loc[1])
        return tx, ty


if __name__ == '__main__':  # Need to have color get_param in loop for updating from signal.
    rospy.init_node('basket_calc', anonymous=False)
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    color = rospy.get_param("basket_color")
    Basket(color, alt_dist=True)
    rospy.logwarn(color)
    rate.sleep()
    rospy.spin()


