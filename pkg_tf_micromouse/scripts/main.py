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
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    rospy.init_node('directions')

    rate = rospy.Rate(20)

<<<<<<< HEAD
    
	#motion_go_straight()
    right_turn()
    #rotate(90)
    #rotate(90)
    #rotate(90)
    #rotate(90)
    #rotate(90)

=======
>>>>>>> f4dce7ed5fb58c336d6ed6b610975075eff4fc2e

    while 1:
        if(Is_Straight_Available()):
            motion_go_straight(0.1)
        elif(Is_Right_Available_Available()):
            motion_go_right()
            break
        else:
            motion_go_straight(0.0)

    

if __name__=='__main__':
    main()


