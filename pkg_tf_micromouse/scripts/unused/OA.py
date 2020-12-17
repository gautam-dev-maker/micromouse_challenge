#! /usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from straight import *
from turns import *
pub = None
sensors = {'RIGHT':0,'FRONT':0,'LEFT':0,'FLEFT':0,'R_LEFT':0}

desired_yaw_ =0
pub=None

def main():
    global pub, current_yaw_
    rospy.init_node('Laser_readings', anonymous=True)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/dd_urdf/laser_scan', LaserScan, clbk_laser)
    while not rospy.is_shutdown():
        motion()
    rospy.spin()

def clbk_laser(msg):
    global sensors
    # 145 degrees sensor divided into 3 sensors
    sensors = {
      'RIGHT' : min(min(msg.ranges[0:20]), 10),
      'FRONT' : min(min(msg.ranges[170:190]), 10),
      'LEFT' : min(min(msg.ranges[329:359]), 10),
      'FLEFT': min(max(msg.ranges[240:300]), 10),
    }

def motion():
    global sensors,pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    msg = Twist()
    linear_x = 0
    angular_z = 0
    current_state = ''
    # 0 0 0
    print('front: {} left: {} right: {}'.format(sensors['FRONT'],sensors['LEFT'],sensors['RIGHT']))

    # LEFT IS AVAILABLE
    if sensors['FLEFT']>0.1:
        current_state = '--- TAKING LEFT ---'
        angular_z=0.3
        linear_x=0.3
    else :
        linear_x=0.2
        angular_z=0

    msg.linear.x=linear_x
    msg.angular.z=angular_z
    pub.publish(msg)
    # if sensors['FRONT'] > 0.15 and sensors['LEFT'] > 0.15 and sensors['RIGHT'] > 0.15 :
    #     current_state = '--- NO OBSTACLES ---'
    #     # motion_go_straight(0.3)
    #     motion_go_linear()

    # # 0 1 0
    # elif sensors['FRONT'] < 0.15 and sensors['LEFT'] > 0.15 and sensors['RIGHT'] > 0.15:
    #     current_state = '--- OBSTACLE AT FRONT ---'                                    
    #     motion_go_straight(0.3)
    #     time.sleep(2)
    #     motion_go_left()
    #     motion_go_straight(0.3)
    # # 0 0 1
    # elif sensors['FRONT'] > 0.15 and sensors['LEFT'] > 0.15 and sensors['RIGHT'] < 0.15:
    #     current_state = '--- OBSTACLE AT RIGHT ---'
    #     motion_go_straight(0.3)
    # # 1 0 0
    # elif sensors['FRONT'] > 0.15 and sensors['LEFT'] < 0.15 and sensors['RIGHT'] > 0.15:
    #     current_state = '--- OBSTACLE AT LEFT ---'
    #     motion_go_straight(0.3)
    # # 0 1 1
    # elif sensors['FRONT'] < 0.15 and sensors['LEFT'] > 0.15 and sensors['RIGHT'] < 0.15:
    #     current_state = '--- OBSTACLE AT FRONT AND RIGHT ---'
    #     motion_go_straight(0.3)
    #     time.sleep(2)
    #     motion_go_left()
    #     motion_go_straight(0.3)
    # # 1 1 0
    # elif sensors['FRONT'] < 0.15 and sensors['LEFT'] < 0.15 and sensors['RIGHT'] > 0.15:
    #     current_state = '--- OBSTACLE AT FRONT AND LEFT ---'
    #     motion_go_straight(0.3)
    #     time.sleep(2)
    #     motion_go_right()
    #     motion_go_straight(0.3)
    # # 1 1 1
    # elif sensors['FRONT'] < 0.15 and sensors['LEFT'] < 0.15 and sensors['RIGHT'] < 0.15:
    #     current_state = '--- OBSTACLE AT FRONT, LEFT AND RIGHT ---'
    #     motion_go_right()
    #     motion_go_right()
    #     motion_go_straight(0.3)
    # # 1 0 1
    # elif sensors['FRONT'] > 0.15 and sensors['LEFT'] < 0.15 and sensors['RIGHT'] < 0.15:
    #     current_state = '--- OBSTACLE AT LEFT AND RIGHT ---'
    #     motion_go_straight(0.3)
    # else:
    #     current_state = 'UNKNOWN CASE'
    #     rospy.loginfo(sensors)
    rospy.loginfo(current_state)
    

if __name__ == '__main__':
    main()




