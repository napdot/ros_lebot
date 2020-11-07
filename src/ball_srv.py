import rospy
from additional_msgs.msg import Depth_BallLocation
from srv.srv import ball_srv, ball_srvResponse


class BallService:
    def __init__(self):
        self.ball_msg = Depth_BallLocation
        self.ball_serv = rospy.Service('/ball_service', self.ball_service_callback)
        self.ball_sub = rospy.Subscriber('/ball', Depth_BallLocation, self.ball_sub_callback, queuesize=1)

    def ball_service_callback(self):
        return ball_srvResponse(
            x=self.ball_msg.x,
            y=self.ball_msg.y,
            d=self.ball_msg.d
        )

    def ball_sub_callback(self, data):
        self.ball_msg.x = data.x
        self.ball_msg.y = data.y
        self.ball_msg.d = data.d


if __name__ == "__main__":
    rospy.init_node('/ball_node_srv')
    BallService()
    rospy.spin()
