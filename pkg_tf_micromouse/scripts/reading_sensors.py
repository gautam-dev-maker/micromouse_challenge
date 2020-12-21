#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

def Average(lst): 
    return sum(lst) / len(lst) 

yaw_=0

def clbk_laser(msg):
    # 145 degrees sensor divided into 3 sensors
    sensors={
        'RIGHT': min(min(msg.ranges[0:71]),10),
        'RIGHT_MAX':min(max(msg.ranges[0:71]),10),
        'FRIGHT': min(min(msg.ranges[72:144]),10),
        'FRIGHT_MAX': min(max(msg.ranges[72:144]),10),
        'FRONT': min(min(msg.ranges[144:216]),10),
        'FLEFT': min(min(msg.ranges[216:288]),10),
        'FLEFT_MAX': min(max(msg.ranges[216:288]),10),
        'LEFT': Average(msg.ranges[288:359]),
        'LEFT_MAX':min(max(msg.ranges[288:359]),10),
    }
    # print("Right: {},Front: {}, Left: {}, FLEFT: {}".format(sensors['RIGHT'],sensors['FRONT'],sensors['LEFT'],sensors['FLEFT']))
    # print("FLEFT: {0:.3f}, FLEFT_MAX: {0:.3f}, left: {0:.3f}, left_max: {0:.3f}".format(sensors['FLEFT'],sensors['FLEFT_MAX'],sensors['LEFT'],sensors['LEFT_MAX']))
    print("LEFT: {0:.3f}".format(sensors['LEFT']))

def main():
    rospy.init_node("reading_sensors")
    sub= rospy.Subscriber('/my_mm_robot/laser/scan',LaserScan,clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()
