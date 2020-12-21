#! /usr/bin/env python 

from main import *
list_stack=[]
steps_list=[]

def backtrack():
    global list_stack,steps_list
    current_list=list_stack.pop()
    last_step=steps_list.pop()
    if last_step=='l':
        while (not is_right_available()):
            go_straight(0.2)
        if len(current_list)==0:
            turn_right()
            backtrack()
        else:
            next_step=current_list.pop()
            if next_step=='r':
                while(is_right_available):
                    go_straight(0.2)
            if next_step=='s':
                turn_left()
            list_stack.append(current_list)
            steps_list.append(next_step)
    if last_step=='r':
        while (not is_left_available()):
            go_straight(0.2)
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
        while(is_left_available or is_right_available):
            go_straight(0.2)
        while(not (is_left_available or is_right_available)):
            go_straight(0.2)
        backtrack()

def dfs():
    while(True):
        if ((not is_left_available()) and (not is_right_available()) and is_straight_available()):
            go_straight(0.2)
            print("condition 1")
        elif (is_left_available() or is_right_available()):
            print("condition 2")
            correct_yaw()
            if not (is_left_available() or is_right_available()):
                continue
            current_list=[]
            if is_straight_available():
                current_list.append('s')
            if is_right_available():
                current_list.append('r')
            if is_left_available():
                current_list.append('l')
            next_step=current_list.pop()
            steps_list.append(next_step)
            list_stack.append(current_list)
            if next_step=='l':
                turn_left()
            if next_step=='r':
                turn_right()
        elif not ((is_straight_available()) or (is_left_available()) or (is_right_available())):
            uturn()
            backtrack()

def main():
    global sensors
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    rospy.init_node('directions')
    dfs()

if __name__=='__main__':
    main()


