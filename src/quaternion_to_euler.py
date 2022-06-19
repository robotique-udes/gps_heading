#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from math import degrees

class QuaternionToEuleur():
    def __init__(self):
        rospy.init_node("Quaternion_to_euler", anonymous=True)
        rospy.Subscriber("/imu/data", Imu, self.imuCB, queue_size=1)

    def imuCB(self, msg):
        orientation_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        orientation_euler = euler_from_quaternion(orientation_quat)
        print("Euler orientation: x=%.2f  y=%.2f  z=%.2f" % (degrees(orientation_euler[0]), 
                                                             degrees(orientation_euler[1]), 
                                                             degrees(orientation_euler[2])))

if __name__ == '__main__':
    node = QuaternionToEuleur()
    rospy.loginfo("quaternion_to_euler ready")
    rospy.spin()