#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from tf import transformations

from directions import *
from turns import *

#global variables 
pub=0

def main():
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) 
    
    rospy.init_node('directions')

    rate = rospy.Rate(20)

    
	#motion_go_straight()
    right_turn()
    #rotate(90)
    #rotate(90)
    #rotate(90)
    #rotate(90)
    #rotate(90)


    rate.sleep()


if __name__=='__main__':
    main()


