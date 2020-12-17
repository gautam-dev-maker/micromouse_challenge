#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from tf import transformations

from directions import *
from turns import *

#global variables 
pub=None
sensors={
    'RIGHT':0,
    'RIGHT_MAX':0,
    'FRIGHT':0,
    'FRIGHT_MAX':0,
    'FRONT':0,
    'FLEFT':0,
    'FLEFT_MAX':0,
    'LEFT':0,
    'LEFT_MAX':0,
    
}


def clbk_laser(msg):
    global sensors
    # 360 / 5 = 72
    # sensor_l = min(min(msg.ranges[305:360]), 10)
    # sensor_c =  min(msg.ranges[180],10)
    # sensor_r = min(min(msg.ranges[0:35]), 10)
    sensors={
        'RIGHT': min(min(msg.ranges[0:72]),10),
        'RIGHT_MAX':min(max(msg.ranges[0:72]),10),
        'FRIGHT': min(min(msg.ranges[72:144]),10),
        'FRIGHT_MAX':min(max(msg.ranges[72:144]),10),
        'FRONT': min(min(msg.ranges[144:216]),10),
        'FLEFT': min(min(msg.ranges[216:288]),10),
        'FLEFT_MAX': min(max(msg.ranges[216:288]),10),
        'LEFT': min(min(msg.ranges[288:359]),10),
        'LEFT_MAX':min(max(msg.ranges[288:359]),10),
    }


def main():
    global sensors
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    rospy.init_node('directions')
    msg=Twist()
    rate = rospy.Rate(20)
    # while not rospy.is_shutdown():
    #     print("right: {:,.3f},fright: {:,.3f}, front: {:,.3f},fleft_max: {:,.3f},fleft: {:,.3f},left_max: {:,.3f},left_min: {:,.3f}".format(sensors['RIGHT'],sensors['FRIGHT'],sensors['FRONT'],sensors['FLEFT_MAX'],sensors['FLEFT'],sensors['LEFT_MAX'],sensors['LEFT']))
    while not rospy.is_shutdown():
        # IF left is available it will take left
        if sensors['LEFT']>0.16 and sensors['LEFT_MAX']>0.16:
            print("left: {}, left_max: {}".format(sensors['LEFT'],sensors['LEFT_MAX']))
            rotate(90,0.225,0.5)
            correct_yaw()
        
        # IF right is available it will take right
        elif sensors['RIGHT']>0.16 and sensors['RIGHT_MAX']>0.16:
            print("Right: {}, Right_max: {}".format(sensors['RIGHT'],sensors['RIGHT_MAX']))
            rotate(-90,0.225,0.5)
            correct_yaw()

        # IF It is a dead end it will take a UTURN
        elif sensors['FRONT']<0.05 and sensors['FLEFT']<0.08 and sensors['FRIGHT']<0.08:
            print("FRONT: {}, FLEFT: {}, FRIGHT: {}".format(sensors['FRONT'],sensors['FLEFT'],sensors['FRIGHT']))
            motion_go_straight(0)
            rotate(90,0,0.5)
            rotate(90,0,0.5)
            correct_yaw()

        # FOR GOING STRAIGHT    
        else :
            print("FRONT: {}, FLEFT: {}, FRIGHT: {}".format(sensors['FRONT'],sensors['FLEFT'],sensors['FRIGHT']))
            motion_go_straight(0.2)
            correct_yaw()
        

if __name__=='__main__':
    main()






























































































































































































































































































































