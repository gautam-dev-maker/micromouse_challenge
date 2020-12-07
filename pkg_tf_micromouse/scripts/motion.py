import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

#global variables
yaw_=0
target=90
kp=1.5
pub=0

def clbk_odom(msg):
    global yaw_
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    yaw_ = euler[2]

def rotate(target):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    current=yaw_
    msg1=Twist()
    target_rad=target*math.pi/180
    rospy.loginfo("turning by angle")
    while target_rad-yaw_>0.05 :
        msg1.linear.x=0
        msg1.angular.z=kp*(target_rad-yaw_)
        msg1.linear.x = 0.3
        pub.publish(msg1)
    msg1.linear.x=0
    pub.publish(msg1)
    rospy.loginfo("turning successful")

def motion_go_straight():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = 0.3
    pub.publish(msg)

def motion_go_left():
    rotate(80)
    motion_go_straight()

def motion_go_right():
    rotate(-90)
    motion_go_straight()