#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from lebot.msg import LineLocation
import cv2
import json
import os
import math
from skimage.draw import line


def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection(L1, L2):
    D = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x, y
    else:
        return False



class Line:
    def __init__(self):
        self.line_location = [0, 0]
        self.line_parameters = {"min": [151, 98, 87], "max": [179, 232, 214]}

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback, queue_size=1)
        self.line_location = rospy.Publisher("/line", LineLocation, queue_size=1)

        self.hsv = np.zeros((480, 640, 3), np.uint16)
        self.thresh = np.zeros((480, 640, 3), np.uint16)

        self.line_message = LineLocation()
        self.kernel = np.ones((3, 3), np.uint8)
        self.hsv = np.zeros((480, 640, 3), np.uint16)
        self.thresh = np.zeros((480, 640, 3), np.uint16)

        self.color_bridge = CvBridge()
        self.kernel_size = 5

        self.mid_line = line(self.mid_line[0], self.mid_line[1])

    def get_my_image_callback(self, data):
        try:
            color_image = self.color_bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
            color_image = np.zeros((480, 640, 3), np.uint16)

        hsv = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)
        self.hsv = cv2.GaussianBlur(hsv, (self.kernel.size, self.kernel_size), 0)
        self.get_thresh()
        self.get_line_location()
        self.update_line_message()
        self.line_location.publish(self.line_message)
        return


    def get_thresh(self):
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.inRange(self.hsv, tuple(self.line_parameters['min']), tuple(self.line_parameters['max']))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        self.thresh = cv2.dilate(thresh, kernel, iterations=1)
        return

    def update_line_message(self):
        self.line_message.x, self.line_message.y = self.transform_location(self.line_location)
        return

    def get_line_location(self):
        minLineLength = 40
        max_length = 0
        maxLineGap = 100
        try:
            edges = cv2.Canny(self.thresh, 50, 150, apertureSize=3)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength, maxLineGap)

            lengths = []
            max_index = -1
            max_p0 = 0
            max_p1 = 0

            for index, l in enumerate(lines):
                x1, y1, x2, y2 = l[0]
                p0 = [x1, y1]
                p1 = [x2, y2]
                length_of_line = np.linalg.norm(p1-p0)
                lengths.append(length_of_line)

                if length_of_line > max_length:
                    max_length = length_of_line
                    max_index = index
                    max_p0 = p0
                    max_p1 = p1

            if (lengths[max_index] > minLineLength) and (len(lines) > 0):
                new_line = line(max_p0, max_p1)
                point = intersection(new_line, self.mid_line)
                if point:
                    if not (0 < point[1] < 480):
                        point[1] = 0
                    if not (0 < point[0] < 640):
                        point[0] = 0

                    self.line_location = point[0], point[1]

                self.line_location = [0, 0]
            else:
                self.line_location = [0, 0]

        except:
            self.line_location = [0, 0]

    def transform_location(self, loc):
        tx = int(np.interp((loc[0]), [0, 640], [-320, 320]))
        ty = int(480 - loc[1])
        return tx, ty


if __name__ == '__main__':  # Need to have color get_param in loop for updating from signal.
    rospy.init_node('line_finder', anonymous=False)
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    Line()
    rate.sleep()
    rospy.spin()


