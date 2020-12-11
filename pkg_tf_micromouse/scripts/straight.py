#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from tf import transformations


pub = None
sensors = {'RIGHT':0,'FRONT':0,'LEFT':0}


weighted_front=0

#pid values

kp=1
ki=0
kd=0

integrat_max=0
integrat_min=0

integrat_state=0

prev_error=0
error=0


def clbk_laser(msg):
    global sensors,weighted_front
    sensors={
        'RIGHT':msg.ranges[0],
        'FRONT':msg.ranges[179],
        'LEFT':msg.ranges[359],
    }
    weighted_front=msg.ranges[125]-msg.ranges[143]


def motion():
    global sensors
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
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

    print("angular_z: {}".format(angular_z))

    msg.linear.x=linear_x
    msg.angular.z=angular_z
    pub.publish(msg)


def main():
    global pub
    global sensors
    rospy.init_node('Laser_readings', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/dd_urdf/laser_scan', LaserScan, clbk_laser)
    while not rospy.is_shutdown():
        motion()
        
        
    rospy.spin()

if __name__ == '__main__':
    main()




