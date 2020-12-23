#! /usr/bin/env python 

from main import *
list_stack=[]
steps_list=[]
coordinate_list=[]
vel=0.3
def backtrack():
    global list_stack,steps_list
    current_list=list_stack.pop()
    last_step=steps_list.pop()
    if last_step=='l':
        while (not is_right_available()):
            go_straight(vel)
        if len(current_list)==0:
            turn_right()
            backtrack()
        else:
            next_step=current_list.pop()
            if next_step=='r':
                while sensors['RIGHTMOST']>0.16:
                    print('LAST STEP L NEXT STEP R SO GOING STRAIGHT')
                    go_straight(vel)
            if next_step=='s':
                turn_left()
            list_stack.append(current_list)
            steps_list.append(next_step)
    if last_step=='r':
        while (not is_left_available()):
            go_straight(vel)
        if len(current_list)==0:
            turn_left()
            backtrack()
        else:
            next_step=current_list.pop()
            if next_step=='s':
                turn_right()
            list_stack.append(current_list)
            steps_list.append(next_step)
    if last_step=='s':
        while not (is_left_available() or is_right_available()):
            print('in the FIRST LOOP in backtrack when last step was S ')
            go_straight(vel)
        while (sensors['RIGHTMOST']>0.16 or sensors['LEFTMOST']>0.16):
            print('in the SECOND LOOP in the backtrack when last step was S')
            go_straight(vel)
        backtrack()

def dfs():
    global coordinate_list,vel
    #sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    while(True):
        #sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
        current_coordinates=[]
        x,y=positn()
        current_coordinates.append(x)
        current_coordinates.append(y)
        if ((not is_left_available()) and (not is_right_available()) and is_straight_available()):
            correct_yaw()
            check_wall()
            print("{0:.2f} {1:.2f}".format(x,y))
	    go_straight(vel)
            print("condition 1")
        elif (is_left_available() or is_right_available()):
            print("condition 2")
            correct_yaw()
            if not (is_left_available() or is_right_available()):
                continue
            if current_coordinates in coordinate_list:
                uturn()
                backtrack()
                print('LOOP FOUND')
                continue
            coordinate_list.append(current_coordinates)
            current_list=[]
            if is_straight_available():
                current_list.append('s')
            if is_right_available():
                current_list.append('r')
                # if sensors['LEFT_MAX']>0.3 and (not is_left_available()):
                    # current_list.append('l')
            if is_left_available():
                # if sensors['RIGHT_MAX']>0.3 and (not is_right_available()):
                    # current_list.append('r')
                current_list.append('l')
            next_step=current_list.pop()
            steps_list.append(next_step)
            list_stack.append(current_list)
            if next_step=='l':
                turn_left()
		correct_yaw()
            if next_step=='r':
                turn_right()
		correct_yaw()
        # elif not ((is_straight_available()) or (is_left_available()) or (is_right_available())):
        elif is_uturn_available():
            print('condition 3')
            uturn()
            backtrack()
        else:
            print('condition 4')
            correct_yaw()
            go_straight(vel)

def main():
    global sensors
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    rospy.init_node('directions')
    dfs()

if __name__=='__main__':
    main()


