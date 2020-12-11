import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
2
#global variables
yaw_=0
kp=2
pub=0
current=0

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

# def rotate(target):
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     yaw_=0
#     target_rad=target*math.pi/180
#     target_rad=target_rad+yaw_
#     if target_rad>3.14:
#         target_rad=target_rad-3.14
#     msg1=Twist()
#     print(yaw_)
#     if target_rad>0 :
#         while target_rad-yaw_>0.01:
#             msg1.angular.z=kp*(target_rad-yaw_)
#             pub.publish(msg1)
#         print(yaw_)
#     if target_rad<0 :
#         final= current+target_rad
#         while yaw_-target_rad>0.01:
#             print(yaw_)
#             msg1.angular.z=-kp*(target_rad-yaw_)
#             pub.publish(msg1)
#     msg1.angular.z=0
#     pub.publish(msg1)

# def filter(angle) :
#     if angle>math.pi:
#         angle=math.pi-angle
#         return angle
#     return angle


def rotate(target):
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=20)
    msg=Twist()
    target_rad=target*math.pi/180
    current=yaw_
    final=current+target_rad
    final=filter(final)
    if target_rad>0:
        while abs(final-yaw_)>0.01:
            msg.angular.z=kp*abs(target_rad-final)
            pub.publish(msg)
    msg.angular.z=0
    pub.publish(msg)




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