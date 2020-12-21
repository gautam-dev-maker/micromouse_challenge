#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from tf import transformations
from main import *


def main():
    # global pub
    # global sensors
    rospy.init_node('Laser_readings', anonymous=True)
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # sub = rospy.Subscriber('/dd_urdf/laser_scan', LaserScan, clbk_laser)
   
        # motion_avoid_left()
    correct_dist()
        
        
    rospy.spin()

if __name__ == '__main__':
    main()




