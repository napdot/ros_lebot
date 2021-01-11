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


class Line:
    def __init__(self):
        self.line_location = [0, 0]
        self.line_parameters = {"min": [151, 98, 87], "max": [179, 232, 214]}

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.get_my_image_callback, queue_size=1)
        self.line_location = rospy.Publisher("/line", LineLocation, queue_size=1)

        self.hsv = np.zeros((480, 650, 3), np.uint16)
        self.thresh = np.zeros((480, 650, 3), np.uint16)

        self.line_message = LineLocation()
        self.kernel = np.ones((3, 3), np.uint8)
        self.hsv = np.zeros((480, 650, 3), np.uint16)
        self.thresh = np.zeros((480, 650, 3), np.uint16)

        self.color_bridge = CvBridge()

    def get_my_image_callback(self, data):
        try:
            color_image = self.color_bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
            color_image = np.zeros((480, 650, 3), np.uint16)

        self.hsv = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)
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
        min_lenght = 40
        try:
            edges = cv2.Canny(self.thresh, 50, 200)
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, max_slider, minLineLength=50, maxLineGap=250)
            lenghts = []
            max_index = -1
            max_lenght = 0

            for index, line in enumerate(lines):
                x1, y1, x2, y2 = line[0]
                p0, p1 = [x1, y1], [x2, y2]
                lenght_of_line = np.sqrt(((p0[0]-p0[1])**2)+((p1[0]-p1[1])**2))
                lenghts.append(lenght_of_line)
                if lenght_of_line > max_lenght:
                    max_lenght = lenght_of_line
                    max_index = index

            if (lenghts[max_index] > min_lenght) and (len(lines) > 0):

                self.line_location = [cX, cY]
            else:
                self.line_location = [0, 480]

        except:
            self.line_location = [0, 480]

    def transform_location(self, loc):
        tx = int(np.interp((loc[0]), [0, 640], [-320, 320]))
        ty = int(480 - loc[1])
        return tx, ty

    def bresenham_march(self, img, p1, p2):
        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]
        # tests if any coordinate is outside the image
        if (
                x1 >= img.shape[0]
                or x2 >= img.shape[0]
                or y1 >= img.shape[1]
                or y2 >= img.shape[1]
        ):  # tests if line is in image, necessary because some part of the line must be inside, it respects the case that the two points are outside
            if not cv2.clipLine((0, 0, *img.shape), p1, p2):
                print("not in region")
                return

        steep = math.fabs(y2 - y1) > math.fabs(x2 - x1)
        if steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # takes left to right
        also_steep = x1 > x2
        if also_steep:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        dx = x2 - x1
        dy = math.fabs(y2 - y1)
        error = 0.0
        delta_error = 0.0
        # Default if dx is zero
        if dx != 0:
            delta_error = math.fabs(dy / dx)

        y_step = 1 if y1 < y2 else -1

        y = y1
        ret = []
        for x in range(x1, x2):
            p = (y, x) if steep else (x, y)
            if p[0] < img.shape[0] and p[1] < img.shape[1]:
                ret.append((p, img[p]))
            error += delta_error
            if error >= 0.5:
                y += y_step
                error -= 1
        if also_steep:  # because we took the left to right instead
            ret.reverse()
        return ret


if __name__ == '__main__':  # Need to have color get_param in loop for updating from signal.
    rospy.init_node('line_finder', anonymous=False)
    myRate = rospy.get_param('lebot_rate')
    rate = rospy.Rate(myRate)
    Line()
    rate.sleep()
    rospy.spin()


