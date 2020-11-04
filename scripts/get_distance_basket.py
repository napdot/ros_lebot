import rospy
import numpy as np
from additional_msgs.msg import Depth_BasketLocation
from std_msgs.msg import Uint8


def depth_to_ball(array, ball):
    x, y, r = ball
    y1, y2, x1, x2 = (y - r), (y + r), (x - r), (x + r)
    dist = np.average(array[y1:y2, x1:x2])  # * depth_scale do it on logic? otherwise pass in message.
    return dist


def image_depth_callback(data):
    msg_data = Depth_BallLocation
    ball = (msg_data.x, msg_data.y, msg_data.r)
    depth_img = msg_data.array  # bitwise mask. Otherwise need to extract pixels and more computation.
    depth_to_ball(depth_img, ball)


if __name__ == '__main__':
    rospy.init_node('/depth_ball')
    depth_ball_sub = rospy.Subscriber("/depth_and_ball_location", Depth_BallLocation, image_depth_callback)
    depth_ball_pub = rospy.Publisher('/ball_distance', Uint8, queue_size=1)
    rospy.spin()