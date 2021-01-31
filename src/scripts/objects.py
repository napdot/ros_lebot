import rospy
import numpy as np
from lebot.msg import Wheel
from lebot.msg import Thrower
from lebot.msg import LineLocation


class theRobot:
    def __init__(self):
        # Value publishing
        self.move = rospy.Publisher('/wheel_values', Wheel, queue_size=1)
        self.throw = rospy.Publisher('/thrower_values', Thrower, queue_size=1)
        self.wheel_msg = Wheel()
        self.thrower_msg = Thrower()

        # Parameters
        self.max_speed = 100
        self.max_thrower = 100
        self.min_thrower = 25

        # Kinetic Model
        self.aKI = np.array([[-1 / np.sqrt(3), -1 / 3, 1 / 3], [1 / np.sqrt(3), -1 / 3, 1 / 3], [0, 2 / 3, 1 / 3]])

        # Movements available ______________________

    def move_manual(self, w1, w2, w3):
        self.publish_wheel(w1, w2, w3)
        return

    def forward(self, magnitude):
        if -1 <= magnitude <= 1:
            w1 = - self.max_speed * magnitude
            w2 = self.max_speed * magnitude
            w3 = 0
            self.publish_wheel(w1, w2, w3)
            return

        else:
            rospy.logwarn('Forward values out of magnitude (-1:1)')
            return

    def backward(self, magnitude):
        if -1 <= magnitude <= 1:
            w1 = self.max_speed * magnitude
            w2 = - self.max_speed * magnitude
            w3 = 0
            self.publish_wheel(w1, w2, w3)
            return

        else:
            rospy.logwarn('Backward values out of magnitude (-1:1)')
            return

    def rotate(self, magnitude):
        if -1 <= magnitude <= 1:
            if magnitude >= 0:  # + which is CCW
                w1 = self.max_speed * magnitude
                w2 = self.max_speed * magnitude
                w3 = self.max_speed * magnitude
                self.publish_wheel(w1, w2, w3)
                return
            else:  # - which is CW
                w1 = - self.max_speed * magnitude
                w2 = - self.max_speed * magnitude
                w3 = - self.max_speed * magnitude
                self.publish_wheel(w1, w2, w3)
                return
        else:
            rospy.logwarn('Rotate values out of magnitude (-1:1)')
            return

    def rotate_at_distance(self, magnitude):
        if -1 <= magnitude <= 1:
            if magnitude >= 0:  # + which is CCW
                w1 = 0
                w2 = 0
                w3 = self.max_speed * magnitude
                self.publish_wheel(w1, w2, w3)
                return
            else:  # - which is CW
                w1 = 0
                w2 = 0
                w3 = - self.max_speed * magnitude
                self.publish_wheel(w1, w2, w3)
                return
        else:
            rospy.logwarn('Rotate at distance values out of magnitude (-1:1)')
            return

    def move_to(self, object, magnitude):
        if -1 <= magnitude <= 1:
            yP, xP = np.array([object.d * np.cos(object.angle_aki), object.d * np.sin(object.angle_aki)])
            m = np.dot(self.aKI, np.array([xP, yP, np.arctan2(yP, xP)]))
            mSer = np.rint(np.multiply(np.multiply(np.divide(m, np.max(np.absolute(m))), self.max_speed), magnitude))
            self.publish_wheel(mSer[0], mSer[1], mSer[2])
            return
        else:
            rospy.logwarn('Move to outside magnitude')
            return

    def throw_to(self, object):
        self.publish_thrower(int(self.thrower_calculation(object.d)))

    def stop_throw(self):
        self.publish_thrower(int(0))

    def stop_move(self):
        self.publish_wheel(int(0), int(0), int(0))

    def thrower_calculation(self, dist):
        ser = int(dist * 0.01352 + 17.2)
        if ser < self.min_thrower:
            ser = int(self.min_thrower)
        if ser > self.max_thrower:
            ser = int(self.max_thrower - 1)
        return ser

    # Values publishing____________________

    def publish_wheel(self, w1, w2, w3):
        wheel_speeds = [w1, w2, w3]
        for speed in wheel_speeds:
            if -self.max_speed <= speed <= self.max_speed:
                rospy.logwarn('Outside max speed range')
                return

        self.wheel_msg.w1, self.wheel_msg.w2, self.wheel_msg.w3 = int(w1), int(w2), int(w3)
        self.move.publish(self.wheel_msg)
        return

    def publish_thrower(self, t1):
        if 0 <= t1 <= self.max_thrower:
            self.thrower_msg.t1 = int(t1)
            self.throw.publish(self.msg)
            return
        else:
            rospy.logwarn('Outside thrower max speed range')
            return


class theBall:
    def __init__(self):
        # Ball detected values
        self.x = -320
        self.y = 480
        self.r = 0
        self.d = 0

        # Camera properties
        self.camera_hfov = 74
        self.camera_width = 640
        self.camera_height = 480

        # Calculated properties
        self.angle = 0 * np.pi / 180
        self.isOrientedPixel = False
        self.isOrientedAngle = False
        self.isDetected = False
        self.angle_aki = 0 * np.pi / 180
        self.isNear = False

        # Options
        self.angleWithDistance = False

        # Parameters
        self.orientation_offset_pixel = 20
        self.orientation_offset_angle = 10 * np.pi / 180
        self.min_dist = 400 # in mm
        self.distance_power = 0.5 # Lower, means more flexibility at farther distances
        self.angle_score = self.angle_score_calc()

    def updateValues(self, x, y, d):
        self.x = x
        self.y = y
        self.d = d
        self.isDetected = self.inView()
        self.angle = self.calcAngle()
        self.isOrientedPixel = self.orientedPixel()
        self.isOrientedAngle = self.orientedAngle()
        self.angle_aki = self.calc_angle()
        return

    def inView(self):
        if (self.x == -320 and self.y == 480) or self.d == 0:
            return False
        else:
            return True

    def calcAngle(self):
        # Angle is in radians
        angle = self.x / (self.camera_width / 2) * (self.camera_hfov / 2) * (3.1415 / 180)
        return angle

    def orientedPixel(self):
        if self.isDetected:
            if -self.orientation_offset_pixel < self.x < self.orientation_offset_pixel:
                return 0
            elif self.orientation_offset_pixel < self.x:
                return 1
            else:
                return -1
        else:
            return False

    def orientedAngle(self):
        if self.angleWithDistance:
            if self.isDetected:
                if - self.orientation_offset_angle < self.angle_score_calc() < self.orientation_offset_angle:
                    return 0
                elif self.angle_score_calc() < self.angle:
                    return 1
                else:
                    return -1
            else:
                return False
        else:
            if self.isDetected:
                if - self.orientation_offset_angle < self.angle < self.orientation_offset_angle:
                    return 0
                elif self.orientation_offset_angle < self.angle:
                    return 1
                else:
                    return -1
            else:
                return False

    def angle_score_calc(self):
        return (1 / np.power((self.d - self.min_dist), self.distance_power)) * self.angle

    def calc_angle(self):
        # Angle is in radians
        camera_mid_angle = self.x / (self.camera_width / 2) * (self.camera_hfov / 2) * (3.1415 / 180)
        angle = (np.pi / 2) - camera_mid_angle
        return angle


class theBasket:
    def __init__(self):
        # Ball detected values
        self.x = -320
        self.y = 480
        self.d = 0

        # Camera properties
        self.camera_hfov = 74
        self.camera_width = 640
        self.camera_height = 480

        # Calculated properties
        self.angle = 0 * np.pi / 180
        self.isOrientedPixel = False
        self.isOrientedAngle = False
        self.isDetected = False
        self.angle_aki = 0 * np.pi / 180
        self.isNear = False

        # Options
        self.angleWithDistance = False

        # Parameters
        self.orientation_offset_pixel = 20
        self.orientation_offset_angle = 10 * np.pi / 180
        self.min_dist = 400 # in mm
        self.distance_power = 0.5 # Lower, means more flexibility at farther distances
        self.angle_score = self.angle_score_calc()

    def updateValues(self, x, y, d):
        self.x = x
        self.y = y
        self.d = d
        self.isDetected = self.inView()
        self.angle = self.calcAngle()
        self.isOrientedPixel = self.orientedPixel()
        self.isOrientedAngle = self.orientedAngle()
        self.angle_aki = self.calc_angle()
        return

    def inView(self):
        if (self.x == -320 and self.y == 480) or self.d == 0:
            return False
        else:
            return True

    def calcAngle(self):
        # Angle is in radians
        angle = self.x / (self.camera_width / 2) * (self.camera_hfov / 2) * (3.1415 / 180)
        return angle

    def orientedPixel(self):
        if self.isDetected:
            if -self.orientation_offset_pixel < self.x < self.orientation_offset_pixel:
                return 0
            elif self.orientation_offset_pixel < self.x:
                return 1
            else:
                return -1
        else:
            return False

    def orientedAngle(self):
        if self.angleWithDistance:
            if self.isDetected:
                if - self.orientation_offset_angle < self.angle_score_calc() < self.orientation_offset_angle:
                    return 0
                elif self.angle_score_calc() < self.angle:
                    return 1
                else:
                    return -1
            else:
                return False
        else:
            if self.isDetected:
                if - self.orientation_offset_angle < self.angle < self.orientation_offset_angle:
                    return 0
                elif self.orientation_offset_angle < self.angle:
                    return 1
                else:
                    return -1
            else:
                return False

    def angle_score_calc(self):
        return (1 / np.power((self.d - self.min_dist), self.distance_power)) * self.angle

    def calc_angle(self):
        # Angle is in radians
        camera_mid_angle = self.x / (self.camera_width / 2) * (self.camera_hfov / 2) * (3.1415 / 180)
        angle = (np.pi / 2) - camera_mid_angle
        return angle
