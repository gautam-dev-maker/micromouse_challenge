#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None
sensors = {'W_RIGHT':0,'W_FRONT':0,'W_LEFT':0,'RIGHT':0,'FRONT':0,'LEFT':0}

def clbk_laser(msg):
    global sensors
    # 145 degrees sensor divided into 3 sensors
    range=[0,0,0,0,0,0,0,0,0,0]
    range[0]=msg.ranges[0]*-5
    range[1]=msg.ranges[1]*-4
    range[2]=msg.ranges[2]*-3
    range[3]=msg.ranges[3]*-2
    range[4]=msg.ranges[4]*-1

    range[5]=msg.ranges[5]*1
    range[6]=msg.ranges[6]*2
    range[7]=msg.ranges[7]*3
    range[8]=msg.ranges[8]*4
    range[9]=msg.ranges[9]*5
    sensors = {
      'W_RIGHT' : (range[0]+range[1]+range[2]+range[3])/14,
      'W_FRONT' : (range[4]+range[5]),
      'W_LEFT' : (range[6]+range[7]+range[8]+range[9])/14,
      'RIGHT' : min(min(msg.ranges[0:4]), 10),
      'FRONT' : min(min(msg.ranges[4:6]), 10),
      'LEFT' : min(min(msg.ranges[6:9]), 10),

      
    }

    # print("{} {} {}".format(sensors['W_RIGHT'],sensors['W_FRONT'],sensors['W_LEFT']))

def motion():
    global sensors
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    msg = Twist()
    
    linear_x = 0
    angular_z = 0
    current_state = ''
    # if sensors['W_RIGHT']>-0.14 and sensors['W_LEFT']<0.14 and sensors['FRONT']>0.12:
    #     print("moving straight")
    #     linear_x=0.2
    #     if sensors['W_FRONT']>0:    #tilting right
    #         angular_z=-0.001
    #     elif sensors['W_FRONT']<0:   #tilting left
    #         angular_z=0.001
    if -0.005<sensors['FRONT']<0.005:
        linear_x=0.2
        if sensors['W_RIGHT']>

        

    # msg.linear.x=linear_x
    # msg.angular.z=angular_z
    # pub.publish(msg)

def main():
    global pub
    global sensors
    rospy.init_node('Laser_readings', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # sub = rospy.Subscriber('/dd_urdf/laser_scan', LaserScan, clbk_laser)
    while not rospy.is_shutdown():
        print("{} {} {}".format(sensors['W_RIGHT'],sensors['W_FRONT'],sensors['W_LEFT']))
        
        motion()
        
        
    rospy.spin()

if __name__ == '__main__':
    main()




