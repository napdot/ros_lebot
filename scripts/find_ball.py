# import rospy
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from geometry_msgs.msg import Point
# from sensor_msgs.msg import CompressedImage
# import json
#
# with open('../color_parameters.json') as f:
#     d = json.load(f)
#     green_parameters = d['green']
#
#
# def detectBall(frame):
#     global cX, cY, cR
#     radius_min = 3
#     radius_max = 30
#     kernel = np.ones((3, 3), np.uint8)
#
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(hsv, tuple(green_parameters['min']), tuple(green_parameters['max']))
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#
#     cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
#                             cv2.CHAIN_APPROX_SIMPLE)[-2]
#     if len(cnts) > 0:
#         c = max(cnts, key=cv2.contourArea)
#         ((x, y), r) = cv2.minEnclosingCircle(c)
#         if (r < radius_max) & (r > radius_min):
#             cX = x
#             cY = y
#             cR = r
#         else:
#             cX = 0   # Can it None?
#             cY = 0
#             cR = 0
#
#         return cX, cY, cR
#
#     else:
#         return 0, 0, 0
#
#
# def image_callback(data):
#     global cX, cY, cR
#     image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
#     cX, cY, cR = detectBall(image)
#     point = Point()
#     point.x = cX
#     point.y = cY
#     point.z = cR
#     pub_point.publish(point)
#
#
# if __name__ == '__main__':
#     global cX, cY, cR
#
#     rospy.init_node('/find_ball', anonymous=False)
#     bridge = CvBridge()
#     img_sub = rospy.Subscriber("/image", CompressedImage, image_callback)
#     pub_point = rospy.Publisher('/ball_location', Point, queue_size=1)
#     rospy.spin()
