#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    # 360 / 5 = 72
    regions = [
        min(min(msg.ranges[0:72]), 10),
        min(min(msg.ranges[72:144]), 10),
        min(min(msg.ranges[144:216]), 10),
        min(min(msg.ranges[216:288]), 10),
        min(min(msg.ranges[288:360]), 10),
    ]
    print('%0.3f %0.3f %0.3f %0.3f %0.3f',regions[0],regions[1],regions[2],regions[3],regions[4])
    print(max(msg.ranges[0:360]))

def main():
    rospy.init_node('reading_laser')
    
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
