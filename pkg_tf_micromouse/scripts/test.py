#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from tf import transformations
from main import *

def callback(msg):
	print(msg.pose.pose.position.x)



def main():
    rospy.init_node("check odom")
    odom_sub=rospy.Subscriber('/odom',Odometry,callback)
    rospy.spin()

if __name__ == '__main__':
    main()





