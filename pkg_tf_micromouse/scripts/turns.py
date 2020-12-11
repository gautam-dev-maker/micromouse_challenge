import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

current_yaw_=0
yaw_ = 0
pub = 0

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
        yaw_ = math.pi - yaw_
    print(yaw_)
    return (yaw_ - current_yaw_)


def rotate(degree):
    global yaw_,current_yaw_
    angular_z = 0.3 if degree>0 else -0.3
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    turn_angle=degree*math.pi/180
    target_yaw = current_yaw_+turn_angle
    rospy.loginfo("turning by angle")
    yaw_error(target_yaw)
    while yaw_error(target_yaw)>0.01 or yaw_error(target_yaw)<-0.01:
        msg.angular.z= angular_z
        msg.linear.x = 0
        pub.publish(msg)
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)
    rospy.loginfo("turning successful")