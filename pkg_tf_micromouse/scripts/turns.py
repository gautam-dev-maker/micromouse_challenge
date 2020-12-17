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

