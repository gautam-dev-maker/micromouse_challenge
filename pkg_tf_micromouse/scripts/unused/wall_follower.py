#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

kp=4
sensors={'LEFT1':0,'LEFT2':0,'FRONT':0}
dist_to_be_maintained=0.1
wall_lead=0.1
pub=None

def clbk_laser(msg):
    global sensors
    sensors = {
    'LEFT1': msg.ranges[359],
    'LEFT2': msg.ranges[269],
    }
    
    print("LEFT1: {}, LEFT2: {}".format(sensors['LEFT1'],sensors['LEFT2']))

def main():
    global sensors,pub,kp,wall_lead
    rospy.init_node("reading_sensors")
    sub= rospy.Subscriber('/my_mm_robot/laser/scan',LaserScan,clbk_laser)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while not rospy.is_shutdown():
        linear_x=0.4
        angular_z=0
        msg=Twist()
        # d1=sensors['LEFT1']
        # d2=sensors['LEFT2']
        # distance_from_wall=(d1+d2)/2
        # current_angle=d2-d1
        # angular_z=kp*(distance_from_wall-dist_to_be_maintained)-current_angle
        # if sensors['FRONT']<0.05:
        #     linear_x=-0.2
        #     angular_z=0
        # msg.linear.x=linear_x
        # msg.angular.z=angular_z
        # pub.publish(msg)
        a=sensors['LEFT2']*math.sin(math.pi/4)-dist_to_be_maintained
        b=sensors['LEFT2']*math.cos(math.pi/4)+wall_lead
        alpha_wall=math.atan2(a,b)
        angular_z=alpha_wall
        msg.linear.x=linear_x
        msg.angular.z=angular_z
        pub.publish(msg)
        print("alpha wall: {}".format(angular_z))


    rospy.spin()

if __name__ == '__main__':
    main()