#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion, QuaternionStamped
from math import pi, degrees, radians
from tf import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from collections import deque
from threading import Lock
from geographiclib.geodesic import Geodesic
from copy import deepcopy

class GpsHeading():
    def __init__(self):
        # ROS stuff
        rospy.init_node("gps_heading", anonymous = True)
        self.gps_sub = rospy.Subscriber('/fix', NavSatFix, self.gpsCB, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imuCB, queue_size=1)
        self.heading_pub = rospy.Publisher('/gps_heading', Imu, queue_size=1)  # TODO: change to pose?
        self.tf = TransformListener()

        # Parameters
        self.publishing_rate = rospy.get_param("~publishing_rate", 10)  # Hz
        self.min_distance = rospy.get_param("~min_distance", 5)  # TODO: ROS parameter
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.gps_positions_queue_length = rospy.get_param("~gps_positions_queue_length", 5)  # Newest positions are added on the left, oldest positions are on the right
        self.orientation_covariance = [2.6030820491461885e-07, 0.0, 0.0, \
                                       0.0, 2.6030820491461885e-07, 0.0, \
                                       0.0, 0.0, 0.0]  # TODO: use better covariance values

        # GPS variables
        self.gps_position_queue = deque(maxlen=self.gps_positions_queue_length)
        self.latest_gps_heading = 0
        self.got_first_gps_heading = False
        self.previous_gps_heading = 0
        self.gps_heading_has_converged = False  # Only switch to GPS heading once it has converged
        self.gps_heading_epsilon = rospy.get_param("~gps_heading_epsilon", 0.0174533)  # Maximum difference in radians between the current gps heading and previous gps heading to consider the heading converged
        self.nb_of_heading_disagreements = 0  # Number of converged GPS headings that have been calculated since the IMU heading was last corrected

        # IMU variable
        self.got_first_imu_orientation = False
        self.previous_imu_orientation = Quaternion()
        self.relative_yaw = 0
        self.latest_imu_euler_orientation = [0, 0, 0]
        self.max_nb_of_heading_disagreements = rospy.get_param("~max_nb_of_heading_disagreements", 3)  # Number of times a disagreement between the IMU heading and the GPS heading will be tolerated before a correction is forced.

        # Other variables
        self.mutex = Lock()  # Mutex is used because the two callbacks run in their own thread but they use shared data
            
    def imuCB(self, msg):
        quat = QuaternionStamped()
        quat.header = msg.header
        quat.quaternion = msg.orientation
        transformed_quat = self.tf.transformQuaternion(self.base_frame, quat)

        if not self.got_first_imu_orientation:
            self.previous_imu_orientation = transformed_quat.quaternion
            self.got_first_imu_orientation = True
            return

        # Get yaw difference between current yaw and previous yaw
        previous_imu_quaternion_orientation = [self.previous_imu_orientation.x, self.previous_imu_orientation.y, \
                                              self.previous_imu_orientation.z, self.previous_imu_orientation.w]
        current_imu_quaternion_orientation = [transformed_quat.quaternion.x, transformed_quat.quaternion.y, transformed_quat.quaternion.z, transformed_quat.quaternion.w]

        previous_imu_euler_orientation = euler_from_quaternion(previous_imu_quaternion_orientation)
        current_imu_euler_orientation = euler_from_quaternion(current_imu_quaternion_orientation)

        self.mutex.acquire()
        try:
            self.relative_yaw += wrap_angle_pi(current_imu_euler_orientation[2] - previous_imu_euler_orientation[2])
            self.relative_yaw = wrap_angle_pi(self.relative_yaw)
            absolute_yaw = wrap_angle_pi(self.relative_yaw + self.latest_gps_heading)
            # print("current heading: %.2f" % degrees(absolute_yaw))
        finally:
            self.mutex.release()

        absolute_quaternion_orientation = quaternion_from_euler(current_imu_euler_orientation[0], current_imu_euler_orientation[1], absolute_yaw)

        heading_msg = deepcopy(msg)
        heading_msg.orientation.x = absolute_quaternion_orientation[0]
        heading_msg.orientation.y = absolute_quaternion_orientation[1]
        heading_msg.orientation.z = absolute_quaternion_orientation[2]
        heading_msg.orientation.w = absolute_quaternion_orientation[3]
        self.heading_pub.publish(heading_msg)

        self.latest_imu_euler_orientation = current_imu_euler_orientation
        self.previous_imu_orientation = transformed_quat.quaternion


    def gpsCB(self, msg):
        # A queue is used instead of only saving one previous position because if we only save one, the robot has to
        # travel the entire threshold distance before a new heading can be calculated. If we save multiple previous 
        # positions in a queue, we can calculate more frequently because one of the few previous positions is bound
        # to be the threshold distance away already.

        # Since GPS is only accurate to the order of a few meters, it is unnecessary to transform it in the base frame

        for position in self.gps_position_queue:
            # Starting from the most recent previous position, find a previous position that is
            # farther than the minimum distance threshold
            distance = calculate_distance(position, (msg.latitude, msg.longitude))

            if distance > self.min_distance:
                yaw = calculate_bearing(position, (msg.latitude, msg.longitude))
                if not self.got_first_gps_heading:
                    rospy.loginfo("Got first gps heading")
                    self.previous_gps_heading = yaw
                    self.got_first_gps_heading = True
                    return
                yaw_diff_from_previous = abs(yaw - self.previous_gps_heading)
                if yaw_diff_from_previous <= self.gps_heading_epsilon:
                    rospy.loginfo("GPS heading converged")
                    self.mutex.acquire()
                    try:
                        self.latest_gps_heading = yaw
                        self.relative_yaw = 0  # Reset relative yaw since a new gps heading has arrived 
                        print("GPS heading: %.2f" % degrees(self.latest_gps_heading))
                    finally:
                        self.mutex.release()
                
                self.previous_gps_heading = yaw
                break
        self.gps_position_queue.appendleft((msg.latitude, msg.longitude))


def wrap_angle_pi(angle):
    # Wraps an angle in radians to -pi : pi
    return (angle + pi) % (2 * pi) - pi

def calculate_bearing(p1, p2):
    # Returns the bearing from point 1 to point 2 (lat/long). 
    # by default, the geodesic azimuth value is in degrees, from -pi to pi, with 0 pointing North.
    # Positive is clockwise and negative is counter-clockwise
    # To match the ROS convention, units have to be in radians, the zero should point east and
    # positive should be counter-clockwise
    return wrap_angle_pi(-radians(Geodesic.WGS84.Inverse(p1[0], p1[1], p2[0], p2[1])['azi1']) + pi/2)

def calculate_distance(p1, p2):
    # Returns the distance in meters between two points (lat/long)
    geod = Geodesic.WGS84
    return geod.Inverse(p1[0], p1[1], p2[0], p2[1])['s12']


if __name__ == '__main__':
    gps_heading = GpsHeading()
    rospy.loginfo("gps_heading ready")
    rospy.spin()
