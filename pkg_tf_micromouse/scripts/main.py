#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

#global variables 
pub=None
yaw_ = 0
pub = 0

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

def clbk_odom(msg):
    global yaw_,current_yaw_
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    current_yaw_ = euler[2]

def yaw_error(target_yaw):
    global current_yaw_ ,yaw_
    yaw_ = target_yaw
    if(-(math.pi)<target_yaw<-3.1 or target_yaw<-(math.pi)):
        yaw_ = -target_yaw
    if(yaw_>math.pi):
        yaw_ = yaw_ - 2*math.pi
    print('current yaw: {} yaw: {}'.format(current_yaw_,yaw_))
    return (yaw_ - current_yaw_)

def rotate(degree,linear_velocity,angular_velocity):
    global yaw_,current_yaw_
    angular_z = angular_velocity if degree>0 else -angular_velocity
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    turn_angle=degree*math.pi/180
    target_yaw = current_yaw_+turn_angle
    print("target_yaw: {} current_yaw:{}".format(target_yaw,current_yaw_))
    rospy.loginfo("turning by angle")
    yaw_error(target_yaw)
    while yaw_error(target_yaw)>0.03 or yaw_error(target_yaw)<-0.03:
        msg.angular.z= angular_z
        msg.linear.x=linear_velocity
        pub.publish(msg)
    msg.angular.z=0
    msg.linear.x=0
    pub.publish(msg)
    rospy.loginfo("turning successful")

def correct_yaw():
    global yaw_,current_yaw_
    yaw_degree=math.degrees(current_yaw_)
    if 0<yaw_degree<=30 or -30<=yaw_degree<0 and yaw_degree!=0:
        rotate(-yaw_degree,0,0.1)
    elif 60<=yaw_degree<=120 and yaw_degree!=90:
        rotate(90-yaw_degree,0,0.1)
    elif 150<=yaw_degree<=180 and yaw_degree!=180:
        rotate(180-yaw_degree,0,0.1)
    elif -180<=yaw_degree<-150 and yaw_degree!=-180:
        rotate(-yaw_degree-180,0,0.1)
    elif -120 <= yaw_degree<=-60 and yaw_degree!=-90 :
        rotate(-90-yaw_degree,0,0.1)

    
def motion_go_straight(linear_velocity):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = linear_velocity
    pub.publish(msg)


def clbk_laser(msg):
    global sensors
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






























































































































































































































































































































