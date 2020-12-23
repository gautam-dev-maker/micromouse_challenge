#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

def Average(lst): 
    return sum(lst) / len(lst) 

#global variables 
pub=None
yaw_ = 0
current_yaw_=0
current_x_=0
current_y_=0

sensors={
    'RIGHTMOST':0,
    'RIGHT':    0,
    'RIGHT_AVG':0,
    'RIGHT_MAX':0,
    'RIGHT_AVG':0,
    'FRIGHT':   0,
    'FRIGHT_MAX':0,
    'FRONT':    0,
    'FLEFT':    0,
    'FLEFT_MAX':0,
    'LEFTMOST':0,
    'LEFT':     0,
    'LEFT_AVG': 0,
    'LEFT_MAX': 0,

}

def clbk_odom(msg):
    global yaw_,current_yaw_,current_x_,current_y_
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    current_x_=round(msg.pose.pose.position.x,2)
    current_y_=round(msg.pose.pose.position.y,2)

    current_yaw_ = euler[2]


def yaw_error(target_yaw):
    global current_yaw_ ,yaw_
    yaw_ = target_yaw
    if(-(math.pi)<target_yaw<-3.1 or target_yaw<-(math.pi)):
        yaw_ = yaw_ + 2*math.pi
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
    while yaw_error(target_yaw)>0.04 or yaw_error(target_yaw)<-0.04:
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
        error=-yaw_degree
        if abs(error)>5:
            rotate(-yaw_degree,0,0.1)
    elif 60<=yaw_degree<=120 and yaw_degree!=90:
        error=90-yaw_degree
        if abs(error)>5:
            rotate(90-yaw_degree,0,0.1)
    elif 150<=yaw_degree<=180 and yaw_degree!=180:
        error=180-yaw_degree
        if abs(error)>5:
            rotate(180-yaw_degree,0,0.1)
    elif -180<=yaw_degree<-150 and yaw_degree!=-180:
        error=-yaw_degree-180
        if abs(error)>5:
            rotate(-yaw_degree-180,0,0.1)
    elif -120 <= yaw_degree<=-60 and yaw_degree!=-90 :
        error=-90-yaw_degree
        if abs(error)>5:
            rotate(-90-yaw_degree,0,0.1)

def is_left_available():
    global sensors
    if sensors['LEFT_AVG']>0.28 or sensors['LEFT']>0.14:
        return True
    return False

def is_right_available():
    global sensors
    if sensors['RIGHT_AVG']>0.28 or sensors['RIGHT']>0.14:
        return True
    return False

def is_straight_available():
    global sensors
    if sensors['FRONT']>0.165:
       return True
    return False
    # return not (is_left_available() or is_right_available() or is_uturn_available())

def is_uturn_available():
    global sensors
    if sensors['FRONT']<0.08 and sensors['FLEFT']<0.08 and sensors['FRIGHT']<0.08:
        return True
    return False

def go_straight(linear_velocity):
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    print("going straight with velocity {}".format(linear_velocity))
    msg = Twist()
    # correct_yaw()
    check_wall()
    msg.linear.x = linear_velocity
    pub.publish(msg)

def turn_right():
    rotate(-90,0.225,0.5)
    # correct_yaw()
    check_wall()
    

def turn_left():
    
    rotate(90,0.225,0.5)
    check_wall()
    # correct_yaw()

    

def uturn():
    check_wall()
    go_straight(0.0)
    rotate(90,0,0.5)
    rotate(90,0,0.5)
    correct_yaw()

    
    


def motion_go_straight(linear_velocity):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = linear_velocity
    pub.publish(msg)


def clbk_laser(msg):
    global sensors
    sensors={
        'RIGHTMOST':msg.ranges[0],
        'RIGHT': min(min(msg.ranges[0:71]),10),
        'RIGHT_MAX':min(max(msg.ranges[0:71]),10),
        'RIGHT_AVG':Average(msg.ranges[0:71]),
        'FRIGHT': min(min(msg.ranges[72:143]),10),
        'FRIGHT_MAX':min(max(msg.ranges[72:143]),10),
        # 'FRONT': min(max(msg.ranges[144:215]),10),
        'FRONT': msg.ranges[179],
        'FLEFT': min(min(msg.ranges[216:287]),10),
        'FLEFT_MAX': min(max(msg.ranges[216:287]),10),
        'LEFTMOST':msg.ranges[359],
        'LEFT': min(min(msg.ranges[288:359]),10),
        'LEFT_AVG':Average(msg.ranges[288:359]),
        'LEFT_MAX':min(max(msg.ranges[288:359]),10),
    }


def avoid_left_wall():
    global sensors,pub
    # sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    correct_yaw()
    msg=Twist()
    print("correcting the wall distance, rotating by 90")
    rotate(90,0,1)
    correct_yaw()
    print("now going straight")
    while sensors['FRONT']<0.06:
        print("front: {}".format(sensors['FRONT']))
        msg.linear.x=-0.05
        pub.publish(msg)
        # if sensors['FRONT'>0.07:
        #     go_straight(0)
        #     break
    msg.linear.x=0
    pub.publish(msg)
    rotate(-90,0,1)
    correct_yaw()

def avoid_right_wall():
    global sensors,pub
    # sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    msg=Twist()
    correct_yaw()
    print("correcting the wall distance, rotating by 90")
    rotate(-90,0,1)
    correct_yaw()
    print("now going straight")
    while sensors['FRONT']<0.07:
        print("front: {}".format(sensors['FRONT']))
        msg.linear.x=-0.05
        pub.publish(msg)
        # if sensors['FRONT'>0.07:
        #     go_straight(0)
        #     break
    msg.linear.x=0
    pub.publish(msg)
    rotate(90,0,1)
    correct_yaw()

def check_left_wall():
    global sensors 
    # sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    correct_yaw()
    if sensors['LEFT']<0.05:
        avoid_left_wall()

def check_right_wall():
    global sensors 
    # sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    correct_yaw()
    if sensors['RIGHT']<0.05:
        avoid_right_wall()

def check_wall():
    global pub
    # sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg=Twist()
    msg.linear.x=0
    pub.publish(msg)
    check_left_wall()
    check_right_wall()

        
    

