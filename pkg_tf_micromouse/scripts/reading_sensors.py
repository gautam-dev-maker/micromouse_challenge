#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

yaw_=0

def clbk_laser(msg):
    # 145 degrees sensor divided into 3 sensors
    sensors = {
      'RIGHT' : min(min(msg.ranges[0:20]), 10),
      'FRONT' : min(min(msg.ranges[170:190]), 10),
      'LEFT' : min(min(msg.ranges[329:359]), 10),
      'FLEFT': min(max(msg.ranges[240:300]), 10),
    }
    print("Right: {},Front: {}, Left: {}, FLEFT: {}".format(sensors['RIGHT'],sensors['FRONT'],sensors['LEFT'],sensors['FLEFT']))

def main():
    rospy.init_node("reading_sensors")
    sub= rospy.Subscriber('/my_mm_robot/laser/scan',LaserScan,clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()
