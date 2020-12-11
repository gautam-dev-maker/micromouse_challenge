#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from tf import transformations

from directions import *
from motion import *

#global variables 
pub=0

def main():
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) 
    
    rospy.init_node('directions')

    rate = rospy.Rate(20)

    # while not rospy.is_shutdown():
    #     if Is_Left_Available():
    #        motion_go_left()
    #     elif Is_Right_Available() :
    #        motion_go_right()
    #     else :
    #         motion_go_straight()
    while not rospy.is_shutdown():
        rotate(90)
        rotate(90)
        break
        


    rate.sleep()


if __name__=='__main__':
    main()


