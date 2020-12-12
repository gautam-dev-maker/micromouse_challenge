import rospy
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from directions import *


current_yaw_=0
yaw_ = 0
pub = 0
sensor_l,sensor_c,sensor_r=0,0,0

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

def clbk_laser(msg):
    global sensor_l,sensor_c,sensor_r
    # 360 / 5 = 72
    sensor_l = msg.ranges[359]
    sensor_c =  msg.ranges[179]
    sensor_r = msg.ranges[0]

def yaw_error(target_yaw):
    global current_yaw_ ,yaw_
    yaw_ = target_yaw
    if(-(math.pi)<target_yaw<-3.1 or target_yaw<-(math.pi)):
        yaw_ = -target_yaw
    if(yaw_>math.pi):
        yaw_ = yaw_ - 2*math.pi
    print('current yaw: {} yaw: {}'.format(current_yaw_,yaw_))
    return (yaw_ - current_yaw_)


def rotate(degree):
    global yaw_,current_yaw_
    angular_z = 0.8 if degree>0 else -0.8
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    turn_angle=degree*math.pi/180
    target_yaw = current_yaw_+turn_angle
    print("target_yaw: {} current_yaw:{}".format(target_yaw,current_yaw_))
    rospy.loginfo("turning by angle")
    yaw_error(target_yaw)
    while yaw_error(target_yaw)>0.02 or yaw_error(target_yaw)<-0.02:
        msg.angular.z= angular_z
        msg.linear.x = 0
        pub.publish(msg)
    msg.angular.z=0
    pub.publish(msg)
    rospy.loginfo("turning successful")

<<<<<<< HEAD
def straight_for_time(t):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    t0 = rospy.Time.now().to_sec()
    stop_time=t0+t
    t1= rospy.Time.now().to_sec()
    while(t1<stop_time):
        countdown=stop_time-t1
        print("countdown {}".format(countdown))
        msg.linear.x=0.4
        msg.angular.z=0
        pub.publish(msg)
        t1= rospy.Time.now().to_sec()
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)

def straight_until_noread_left():
    global sensor_l,sensor_c,sensor_r
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    while(sensor_l<0.09):
        print("left :{}".format(sensor_l))
        msg.linear.x=0.3
        msg.angular.z=0
        pub.publish(msg)
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)

def straight_until_noread_right():
    global sensor_l,sensor_c,sensor_r
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    while(sensor_r<0.09):
        print("right :{}".format(sensor_r))
        msg.linear.x=0.3
        msg.angular.z=0
        pub.publish(msg)
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)

def straight_until_read_left():
    global sensor_l,sensor_c,sensor_r
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    #print("sensorleft :{}".format(sensor_l))
    while(True):
        print("left :{}".format(sensor_l))
        if sensor_l<0.9:
            break
        msg.linear.x=0.3
        msg.angular.z=0
        pub.publish(msg)
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)

def straight_until_read_right():
    global sensor_l,sensor_c,sensor_r
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    print("sensorright :{}".format(sensor_r))
    while(sensor_r>0.09):
        print("right :{}".format(sensor_r))
        msg.linear.x=0.3
        msg.angular.z=0
        pub.publish(msg)
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)


def left_turn():
    straight_until_noread_left()
    radius=0.08793
    v=0.1
    w=v/radius
    duration=math.pi/(2*w)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x=4*v
    msg.angular.z=0.96*w
    pub.publish(msg)
    t0 = rospy.Time.now().to_sec()
    stop_time=(t0+duration)
    t1=  rospy.Time.now().to_sec()
    while(t1<stop_time):
        msg.linear.x=4*v
        msg.angular.z=0.96*w
        print("v={} w={}".format(v,w))
        pub.publish(msg)
        t1=  rospy.Time.now().to_sec()
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)
    rate=rospy.Rate(5)
    rate.sleep()
    straight_for_time(0.5)

def right_turn():
    straight_until_noread_right()
    radius=0.08793
    v=0.2
    w=-v/radius
    duration=abs(math.pi/(2*w))
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x=4*v
    msg.angular.z=0.96*w
    pub.publish(msg)
    t0 = rospy.Time.now().to_sec()
    stop_time=(t0+duration)
    t1=  rospy.Time.now().to_sec()
    while(t1<stop_time):
        msg.linear.x=4*v
        msg.angular.z=0.96*w
        print("v={} w={}".format(v,w))
        pub.publish(msg)
        t1=  rospy.Time.now().to_sec()
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)
    straight_for_time(0.5)
    
=======

def motion_go_straight(linear_velocity):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = linear_velocity
    pub.publish(msg)

<<<<<<< HEAD
def motion_go_left(linear_velocity):
    # rotate(90)
    # motion_go_straight(0.15)
    # time.sleep(3)
    # motion_go_straight(0.0)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = linear_velocity
    msg.angular.z= 0.3
    pub.publish(msg)
    time.sleep(1.5)
    
=======
def motion_go_left():
    #time.sleep(2.5)
    rotate(90)
    motion_go_straight(0.15)
    #time.sleep(3)
    #motion_go_straight(0.0)
>>>>>>> f4dce7ed5fb58c336d6ed6b610975075eff4fc2e

def motion_go_right():
    #time.sleep(2.5)
    rotate(-90)
    motion_go_straight(0.15)
    #time.sleep(3)
    #motion_go_straight(0.0)
>>>>>>> f4dce7ed5fb58c336d6ed6b610975075eff4fc2e
