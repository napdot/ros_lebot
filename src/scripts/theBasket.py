#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from lebot.msg import Depth_BasketLocation
import cv2
import json


class Basket:
    def __init__(self, color, alt_dist):
        self.color = color
        self.red_parameters = []
        self.blue_parameters = []
        self.basket_location = [0, 0]
        self.basket_distance = 0
        self.basket_edges = [0, 0, 0, 0]

        self.alt_dist = alt_dist

        self.set_basket_parameters()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_my_depth_callback)
        self.basket_depth_location_pub = rospy.Publisher("basket", Depth_BasketLocation, queue_size=10)
        self.hsv = np.zeros((480, 650, 3), np.uint16)
        self.thresh = np.zeros((480, 650, 3), np.uint16)
        self.depth = np.zeros((480, 650, 3), np.uint16)
        self.basket_message = Depth_BasketLocation()
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
        self.basket_message.x = int(self.basket_location[0])
        self.basket_message.y = int(self.basket_location[1])
        self.basket_message.d = int(self.basket_distance)
        return

    def get_basket_location(self):
        min_basket_area = 50
        try:
            cnt = cv2.findContours(self.thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            areas = [cv2.contourArea(c) for c in cnt]
            max_index = np.argmax(areas)
            contour = cnt[max_index]
            if cv2.contourArea(contour) > min_basket_area:
                x1, y1, w, h = cv2.boundingRect(contour)
                x2, y2 = x1 + w, y1 + h
                self.basket_edges = [x1, x2, y1, y2]
                M = cv2.moments(contour)
                cX, cY =(int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if cX > 480:
                    cX = 480
                if cX < 0:
                    cX = 0
                if cY > 640:
                    xY = 640
                if cY < 0:
                    cY = 0
                self.basket_distance = [cX, cY]
            else:
                self.basket_location = [0, 0]

        except:
            self.basket_location = [0, 0]

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
    color = rospy.get_param("basket_color")
    Basket(color, alt_dist=True)
    rospy.spin()


