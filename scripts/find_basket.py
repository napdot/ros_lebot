import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from additional_msgs.msg import Rect
from sensor_msgs.msg import CompressedImage
import json


with open('../color_parameters.json') as f:
    d = json.load(f)
    blue_parameters = d['blue']
    red_parameters = d['red']


def detectBasket(frame, color_parameters):
    kernel = np.ones((3, 3), np.uint8)
    hsv = cv2.cvtColor(frame)
    mask = cv2.inRange(hsv, tuple(color_parameters['min']), tuple(color_parameters['max']))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    basket_contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt = basket_contours[0]
    x1, y1, w, h = cv2.boundingRect(cnt)
    x2, y2 = x1 + w, y1 + h
    return x1, x2, y1, y2


def image_callback(data):
    image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    x1, x2, y1, y2 = detectBasket(image, using_color)
    rect = Rect()
    rect.x1 = x1
    rect.x2 = x2
    rect.y1 = y1
    rect.y2 = y2
    pub_point.publish(rect)


if __name__ == '__main__':
    global using_color
    using_color = blue_parameters # or red_parameters
    rospy.init_node('/find_basket', anonymous=False)
    bridge = CvBridge()
    img_sub = rospy.Subscriber("/image", CompressedImage, image_callback)
    pub_point = rospy.Publisher('/basket_location', Rect, queue_size=1)
    rospy.spin()
