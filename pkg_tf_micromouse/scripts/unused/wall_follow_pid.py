#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None
sensors = {'RIGHT':0,'FRONT':0,'LEFT':0}
kp=0
ki=0
kd=0
proportionalerror=0
prev_error=0
cummulative_error=0
difference_error=0
pid_correction=0

def clbk_laser(msg):
    global sensors
    sensors={
        'RIGHT': min(msg.ranges[0],10),
        'FRONT': min(msg.ranges[179],10),
        'LEFT' : min(msg.ranges[359],10)
    }

def wallfollow():
    global sensors,prev_error,proportionalerror,cummulative_error,difference_error,pid_correction,kp,ki,kd
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    proportionalerror = sensors['RIGHT']-sensors['LEFT']
    cummulative_error+=proportionalerror
    if(cummulative_error>10):
        cummulative_error = 10
    if(cummulative_error<0.001):
        cummulative_error = 0.0001
    difference_error = proportionalerror-prev_error
    pid_correction = kp*proportionalerror+ki*cummulative_error+kd*difference_error
    prev_error = pid_correction
    print("proportional: {} cummulative: {} difference: {} correction: {}".format(proportionalerror,cummulative_error,difference_error,pid_correction))
    go_straight()

def go_straight():
    global pid_correction,sensors
    global pub
    msg=Twist()
    d=0.15
    if(sensors['FRONT']>d and sensors['LEFT']<d and sensors['RIGHT']<d):
        msg.linear.x=0.1
        msg.angular.z=pid_correction

    pub.publish(msg)

def main():
    global pub
    rospy.init_node('Laser_readings', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/dd_urdf/laser_scan', LaserScan, clbk_laser)
    while 1:
        wallfollow()
    rospy.spin()


if __name__ == '__main__':
    main()