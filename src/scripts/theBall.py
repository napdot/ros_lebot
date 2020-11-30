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
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_my_depth_callback)
        self.ball_depth_location_pub = rospy.Publisher("ball", Depth_BallLocation, queue_size=1)
        self.mask = None
        self.ball_message = Depth_BallLocation()

    def get_my_image_callback(self, data):
        bridge_img = CvBridge()
        image = bridge_img.compressed_imgmsg_to_cv2(data, "bgr8")
        self.get_ball_location(image)

    def get_my_depth_callback(self, data):
        bridge_depth = CvBridge()
        depth_image = bridge_depth.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.depth_to_ball(depth_image)
        self.ball_message.x = 0
        self.ball_message.y = 0
        self.ball_message.d = 0
        self.update_ball_message()
        self.ball_depth_location_pub.publish(self.ball_message)

    def depth_to_ball(self, depth_img):
        # y1, y2, x1, x2 = (self.ball_location[1] - self.ball_location[2]),\
        #                  (self.ball_location[1] + self.ball_location[2]),\
        #                  (self.ball_location[0] - self.ball_location[2]),\
        #                  (self.ball_location[0] + self.ball_location[2])
        selected_depth_array = depth_img * self.mask
        self.ball_distance = np.mean(selected_depth_array)

    def update_ball_message(self):
        self.ball_message.x = self.ball_location[0]
        self.ball_message.y = self.ball_location[1]
        self.ball_message.d = self.ball_distance

    def get_ball_location(self, frame):
        radius_min = 10
        radius_max = 50
        kernel = np.ones((3, 3), np.uint8)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv, tuple(self.green_parameters['min']), tuple(self.green_parameters['max']))
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel)

        cnts = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), r) = cv2.minEnclosingCircle(c)
            if (r < radius_max) & (r > radius_min):
                cX = x
                cY = y
            else:
                cX = 0
                cY = 0

            self.ball_location = [cX, cY]

        else:
            self.ball_location = [0, 0]

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

