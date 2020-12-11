#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

yaw_=0
weighted_front=0

def clbk_laser(msg):
    global weighted_front
    # weighted_front=msg.ranges[125]-msg.ranges[143]
    # print("Weighted Front : {}".format(weighted_front))
    print("right: {}, front: {}, left: {}".format(msg.ranges[0],msg.ranges[179],msg.ranges[359]))

def clbk_odom(msg):
    global yaw_
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    yaw_ = euler[2]
    # print("yaw:- {}".format(yaw_))

def main():
    rospy.init_node('reading_laser')
    
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rospy.spin()

if __name__ == '__main__':
    main()
