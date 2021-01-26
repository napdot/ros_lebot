#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from lebot.msg import Depth_BallLocation
import cv2
import json
import math
import os


class Ball:
    def __init__(self, alt_dist):
        self.green_parameters = []
        self.ball_location = [0, 0]
        self.ball_distance = 0
        self.ball_radius = 0

        self.alt_dist = alt_dist

        self.set_ball_parameters()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_my_depth_callback, queue_size=1)
        self.ball_depth_location_pub = rospy.Publisher("ball", Depth_BallLocation, queue_size=1)
        self.hsv = np.zeros((480, 640, 3), dtype=np.float32)
        self.thresh = np.zeros((480, 640, 3), dtype=np.float32)
        self.depth = np.zeros((480, 640, 3), dtype=np.float32)
        self.ball_message = Depth_BallLocation()
        self.kernel = np.ones((3, 3), np.uint8)
        self.depth_bridge = CvBridge()
        self.color_bridge = CvBridge()
        self.thrower_mask = self.gen_thrower_mask()

    def gen_thrower_mask(self):
        t_mask = np.ones((480, 640), np.uint8)
        t_mask[325:,192:480] = 0
        t_mask[330:,:50] = 0
        t_mask[330:, 580:] = 0
        t_mask[400:,] = 0
        t_mask[280:,370:450] = 0
        return t_mask

    def get_my_image_callback(self, data):
        try:
            color_image = self.color_bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
            color_image = np.zeros((480, 640, 3), np.uint16)

        self.hsv = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)
        self.hsv = cv2.GaussianBlur(self.hsv,(5,5),0)
        self.get_thresh()
        return

    def get_my_depth_callback(self, data):
        try:
            self.depth = np.array(self.depth_bridge.imgmsg_to_cv2(data, desired_encoding="passthrough"), dtype=np.float32)
        except CvBridgeError as e:
            print(e)
            self.depth = np.zeros((480, 640), np.float32)


        self.get_ball_location()
        self.get_depth_to_ball()
        self.update_ball_message()
        self.ball_depth_location_pub.publish(self.ball_message)
        return

    def get_thresh(self):
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.inRange(self.hsv, tuple(self.green_parameters['min']), tuple(self.green_parameters['max']))
        thresh = np.multiply(thresh, self.thrower_mask)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        self.thresh = cv2.dilate(thresh, kernel, iterations=1)
        return

    def get_depth_to_ball(self):
        self.ball_distance = 0
        try:
            self.ball_distance = np.mean(self.thresh * self.depth)
        except:
            self.ball_distance = 0

        if self.alt_dist:
            try:
                self.ball_distance = self.depth[int(self.ball_location[1]), int(self.ball_location[0])]
            except:
                self.ball_distance = 0

        if self.ball_location[0] == 0 or self.ball_location[1] == 0:
            self.ball_distance = 0
        return


    def update_ball_message(self):
        x, y = self.transform_location(self.ball_location)
        self.ball_message.x, self.ball_message.y = int(x), int(y)
        self.ball_message.d = int(self.ball_distance)
        return

    def get_ball_location(self):
        area_min = 35
        area_max = 6100
        self.ball_location = [0, 0]
        try:
            cnts = cv2.findContours(self.thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if len(cnts) != 0:
                c = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(c)
                # rospy.logwarn(area)
                if area_max > area > area_min:
                    ((cX, cY), cR) = cv2.minEnclosingCircle(c)
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

    def set_ball_parameters(self):
        try:
            dir = '../test_lebot/src/ros_lebot/src/scripts/color_parameters.json'
            with open(dir) as f:
                d = json.load(f)
                self.green_parameters = d['green']
                rospy.loginfo('Ball masking parameters set')

        except:
            rospy.logwarn("Error reading json - Using defaults (Ball)")
            self.green_parameters = {"min": [51, 131, 0], "max": [81, 226, 255]}
        return

    def transform_location(self, loc):
        tx = int(np.interp((loc[0]), [0, 640], [-320, 320]))
        ty = int(480 - loc[1])
        return tx, ty


if __name__ == '__main__':
    rospy.init_node('ball_calc', anonymous=False)
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    Ball(alt_dist=True)
    rate.sleep()
    rospy.spin()

