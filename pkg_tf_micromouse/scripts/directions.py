import rospy

from sensor_msgs.msg import LaserScan

# distance between each walls
d=0.16

# minimum distance between walls and bot for straight
d_min=0.07

# global variables
sensor_l,sensor_c,sensor_r=0,0,0
pub=0

def clbk_laser(msg):
    global sensor_l,sensor_c,sensor_r
    # 360 / 5 = 72
    sensor_l = min(min(msg.ranges[305:360]), 10)
    sensor_c =  min(msg.ranges[180],10)
    sensor_r = min(min(msg.ranges[0:35]), 10)
    
def Is_Right_Available():
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    if sensor_r>d:
        rospy.loginfo("Right is available")
        return True
    return False

def Is_Left_Available():
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    if sensor_l>d :
        rospy.loginfo("left is available")
        return True
    return False
    
def Is_Straight_Available():
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    if sensor_c>d_min:
        rospy.loginfo("Straight is availabe")
        return True
    return False
    

