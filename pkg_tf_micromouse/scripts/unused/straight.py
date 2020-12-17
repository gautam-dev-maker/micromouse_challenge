#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from tf import transformations


pub = None
sensors = {'RIGHT':0,'FRONT':0,'LEFT':0}

yaw_=0
weighted_front=0

is_straight=False

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

def clbk_laser(msg):
    global sensors,weighted_front
    # sensors={
    #     'RIGHT':msg.ranges[0],
    #     'FRONT':msg.ranges[179],
    #     'LEFT':msg.ranges[359],
    # }
    sensors={
        'RIGHT':msg.ranges[0],
        'FRONT':msg.ranges[179],
        'LEFT': msg.ranges[359],
    }
    weighted_front=msg.ranges[125]-msg.ranges[143]


def front_obst(front):
    if front<0.12 :
        return True
    else :
        return False

def filter(value):
    value=value/0.16803
    return 0.16803*value

def motion_avoid_left():
    global sensors,yaw_
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    msg=Twist()
    msg.linear.x=0
    angular_z=0
    k=sensors['RIGHT']+sensors['LEFT']
    angle=((k**2)-(filter(k)**2))
    if angle>=0 and k!=0:
        angle=angle**0.5
        angle=angle/k
        angle=math.asin(angle)
        if sensors['RIGHT']<sensors['LEFT']:
            angular_z=-angle
        else :
            angular_z=angle
    msg.angular.z=angular_z
    print("angular_z: {}".format(angular_z))
    pub.publish(msg)
    
def motion_go_linear():
    global sensors,yaw_,is_straight
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) 
    
    msg = Twist()
    linear_x=0.2
    angular_z=0
    k=sensors['RIGHT']+sensors['LEFT']
    angle=((k**2)-(0.16803**2))
    if angle>=0:
        angle=angle**0.5
        angle=angle/k
        angle=math.asin(angle)
        if sensors['RIGHT']<sensors['LEFT']:
            angular_z=angle
        else :
            angular_z=-angle
        is_straight=False
    else :
        is_straight=True

    if is_straight:

        if sensors['RIGHT']<0.0553:
            angular_z=0.1
    
        if sensors['LEFT']<0.0553:
            angular_z=-0.1

    
    print("angular_z: {}".format(angular_z))

    msg.linear.x=linear_x
    msg.angular.z=angular_z
    pub.publish(msg)


def main():
    global pub
    global sensors
    rospy.init_node('Laser_readings', anonymous=True)
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # sub = rospy.Subscriber('/dd_urdf/laser_scan', LaserScan, clbk_laser)
    while not rospy.is_shutdown():
        # motion_avoid_left()
        motion_go_linear()
        
        
    rospy.spin()

if __name__ == '__main__':
    main()




