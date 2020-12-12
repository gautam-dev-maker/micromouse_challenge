#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from tf import transformations

from directions import *
from turns import *

#global variables 
pub=0
desired_yaw=0
ros_yaw=0


# def adjust_yaw(angle):
#     desired_yaw=desired_yaw+angle
#     if(-(math.pi)<desired_yaw<-3.1 or desired_yaw<-(math.pi)):
#         ros_yaw = -desired_yaw
#     if(yaw_>math.pi):
#         ros_yaw = math.pi - yaw_

def clbk_odom(msg):
    global ros_yaw
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    ros_yaw= euler[2]

def main():
    global ros_yaw,desired_yaw
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) 
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    rospy.init_node('directions')

    rate = rospy.Rate(20)

<<<<<<< HEAD
    # while not rospy.is_shutdown():
    #     print("desired_yaw:- {}, ros_yaw:- {}".format(desired_yaw,ros_yaw))
    #     if desired_yaw-ros_yaw>0.01 or desired_yaw-ros_yaw<-0.01:
    #         rotate((desired_yaw-ros_yaw)*180/math.pi)
    # # rotate(45)
    # # rotate(45)
    while not rospy.is_shutdown():
        left_turn()
        
        
=======
    
	#motion_go_straight()
    right_turn_rotate()
    #rotate(90)
    #rotate(90)
    #rotate(90)
    #rotate(90)
    #rotate(90)


    #while 1:
    #    if(Is_Straight_Available()):
    #        motion_go_straight(0.1)
    #    elif(Is_Right_Available_Available()):
    #        motion_go_right()
    #       break
    #    else:
    #        motion_go_straight(0.0)
>>>>>>> f4d665635f66fb57d7fe771e7976c361c29a5862

    

if __name__=='__main__':
    main()


