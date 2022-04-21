#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion, QuaternionStamped
from math import sqrt, atan2, pi
from tf import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from collections import deque
from threading import Lock
from geographiclib.geodesic import Geodesic

class GpsHeading():
    def __init__(self):
        # ROS stuff
        rospy.init_node("gps_heading", anonymous = True)
        self.gps_sub = rospy.Subscriber('/fix', NavSatFix, self.gpsCB)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imuCB)
        self.heading_pub = rospy.Publisher('/gps_heading', Imu, queue_size=1)  # TODO: change to pose?
        self.tf = TransformListener()

        # Parameters
        self.publishing_rate = rospy.get_param("~publishing_rate", 10)  # Hz
        self.min_distance = rospy.get_param("~min_distance", 3)  # TODO: ROS parameter
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
        self.gps_heading_epsilon = rospy.get_param("~gps_heading_epsilon", 0.0523599)  # Maximum difference in radians between the current gps heading and previous gps heading to consider the heading converged
        self.nb_of_heading_disagreements = 0  # Number of converged GPS headings that have been calculated since the IMU heading was last corrected

        # IMU variable
        self.got_first_imu_orientation = False
        self.previous_imu_orientation = Quaternion()
        self.relative_yaw = 0
        self.latest_imu_euler_orientation = [0, 0, 0]
        self.max_nb_of_heading_disagreements = rospy.get_param("~max_nb_of_heading_disagreements", 3)  # Number of times a disagreement between the IMU heading and the GPS heading will be tolerated before a correction is forced.

        # Other variables
        self.heading_msg = Imu()
        self.mutex = Lock()  # Mutex is used because the two callbacks run in their own thread but they use shared data

        # Initialization process
        self.heading_msg.header.frame_id = self.base_frame
        self.heading_msg.orientation_covariance = self.orientation_covariance

    def run(self):
        try:
            r = rospy.Rate(self.publishing_rate)
            while not rospy.is_shutdown():
                self.mutex.acquire()
                try:
                    self.heading_msg.header.stamp = rospy.Time.now()
                    self.heading_pub.publish(self.heading_msg)
                finally:
                    self.mutex.release()
                r.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass
            
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

        self.relative_yaw += wrap_angle_pi(current_imu_euler_orientation[2] - previous_imu_euler_orientation[2])
        self.relative_yaw = wrap_angle_pi(self.relative_yaw)
        absolute_yaw = wrap_angle_pi(self.relative_yaw + self.latest_gps_heading)

        absolute_quaternion_orientation = quaternion_from_euler(current_imu_euler_orientation[0], current_imu_euler_orientation[1], absolute_yaw)

        self.mutex.acquire()
        try:
            self.latest_imu_euler_orientation = current_imu_euler_orientation
            self.heading_msg.orientation.x = absolute_quaternion_orientation[0]
            self.heading_msg.orientation.y = absolute_quaternion_orientation[1]
            self.heading_msg.orientation.z = absolute_quaternion_orientation[2]
            self.heading_msg.orientation.w = absolute_quaternion_orientation[3]
        finally:
            self.mutex.release()
        # Rest of IMU msg is left empty since we only care about the orientation

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
            distance = calculate_distance(msg.latitude, msg.longitude, position[0], position[1])

            if distance > self.min_distance:
                yaw = calculate_bearing(msg.latitude, msg.longitude, position[0], position[1])
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
                        if abs(self.relative_yaw) <= self.gps_heading_epsilon or self.nb_of_heading_disagreements >= self.max_nb_of_heading_disagreements:
                            rospy.loginfo("Correcting IMU heading with GPS heading")
                            self.nb_of_heading_disagreements = 0
                            # Only use/update GPS heading if it has converged (aka the robot is moving in a straight line). Otherwise, use IMU
                            self.latest_gps_heading = yaw 
                            quaternion_orientation = quaternion_from_euler(self.latest_imu_euler_orientation[0],  self.latest_imu_euler_orientation[1], \
                                                                        self.latest_gps_heading)

                            self.heading_msg.orientation.x = quaternion_orientation[0]
                            self.heading_msg.orientation.y = quaternion_orientation[1]
                            self.heading_msg.orientation.z = quaternion_orientation[2]
                            self.heading_msg.orientation.w = quaternion_orientation[3]     
                            # Rest of IMU msg is left empty since we only care about the orientation

                            self.relative_yaw = 0  # Reset relative yaw since a new gps heading has arrived
                        else:
                            self.nb_of_heading_disagreements += 1
                            rospy.loginfo("Number of heading disagreements: %d" % self.nb_of_heading_disagreements)
                    finally:
                        self.mutex.release()
                
                self.previous_gps_heading = yaw
                break
        self.gps_position_queue.appendleft((msg.latitude, msg.longitude))


def wrap_angle_pi(angle):
    # Wraps an angle in radians to -pi : pi
    return (angle + pi) % (2 * pi) - pi

def calculate_bearing(p1, p2):
    # Returns the bearing from point 1 to point 2 (lat/long). Angle is in degrees, from -180 to 180, with 0 pointing North.
    # Positive is clockwise and negative is counter clockwise
    geod = Geodesic.WGS84
    return geod.Inverse(p1[0], p1[1], p2[0], p2[1])['azi1']

def calculate_distance(p1, p2):
    # Returns the distance in meters between two points (lat/long)
    geod = Geodesic.WGS84
    return geod.Inverse(p1[0], p1[1], p2[0], p2[1])['s12']


if __name__ == '__main__':
    # gps_heading = GpsHeading()
    # rospy.loginfo("gps_heading ready")
    # gps_heading.run()

    geod = Geodesic.WGS84
    p1 = (45.377886, -71.920422)
    p2 = (45.377886, -71.920417)
    g = geod.Inverse(p1[0], p1[1], p2[0], p2[1])
    azimuth = g['azi1']
    print(azimuth)