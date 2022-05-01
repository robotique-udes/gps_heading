# gps_heading
Calculates absolute heading by using sequences of GNSS fixes and gyroscope data.<br><br>
Gyroscopes provide data at a high frequency but drift over time, while GPSs publish at a low rate but do not accumulate drift over time. It is possible to calculate the heading of a robot by calculating the orientation of the line given by a sequence of two GNSS fixes. This angle is then used to correct the orientation of the gyroscope and thus eliminated the accumulated drift. The heading calculated with GNSS fixes is only accurate when the robot is moving in a straight line, so the gyroscope will only be corrected if two orientations calculated one after the other are essentially the same.<br><br>
This technique would not work for if the robot is subject to lateral drift, as would be the case if the robot operates on water or in the air.

**Important**: It is necessary for the robot to be pointing east during the initialization of this node since that is where the orientation origin is.

## Subscribed topics
* fix (sensor_msgs/NavSatFix): GNSS fix of the robot
* imu/data (sensor_msgs/Imu): Imu data of the robot

## Published topics
* gps_heading (sensor_msgs/Imu): Same Imu message as the one coming from the subscribed topics, but with a corrected, absolute orientation.

## Parameters
* ~min_distance (float, default: 5): Minimum distance in meters between two sequential GNSS fixes before calculating the heading.
* ~base_frame (string, default: "base_link"): Name of the base frame.
* ~gps_positions_queue_length (int, default: 5): Length of the queue of GNSS fixes to consider for heading calculations.
* ~gps_heading_epsilon (float, default: 0.0174533): Tolerance in radians for the acceptable difference in orientation between to calculated headings to consider that the rover was moving in a straight line.

## TODO
* Take into account if the robot is going in reverse.
* Use Gyroscope data to check if the robot was moving in a straight line before accepting the corretion.
