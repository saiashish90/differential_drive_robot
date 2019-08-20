#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math
import random
import sys

t = 0
direction = 1
direction2 = 4

#range_
range_ = 2
grid_size = 10
grid_sizey = 10

explore = []
for i in range(grid_size):
    temp = []
    for j in range(grid_sizey):
        temp.append(0)
    explore.append(temp)

active_ = True

# robot state variables
position_ = Point()
position2_ = Point()
curr_point = Point()
yaw_ = 0
yaw2_ = 0

action = 1

# machine state

state_ = 0
state2_= 0

# goal
desired_position_ = Point()
desired_position2_= Point()
initial_position = Point()
initial_position2 = Point()
initial_position.x = 0
initial_position.y = 0
initial_position.z = 0

initial_position2.x = 7
initial_position2.y = 7
initial_position2.z = 0

desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0

desired_position2_.x = 3
desired_position2_.y = 3
desired_position2_.z = 0


# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = .1

# publishers
pub = None
pub2 = None

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_odom2(msg):
    global position2_
    global yaw2_

    # position
    position2_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw2_ = euler[2]

def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def change_state2(state):
    global state2_
    state2_ = state
    print 'State2 changed to [%s]' % state2_

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    #rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        #print 'Yaw error: [%s]' % err_yaw
        change_state(1)

def fix_yaw2(des_pos):
    global yaw2_, pub2, yaw_precision_, state2_
    desired_yaw = math.atan2(des_pos.y - position2_.y, des_pos.x - position2_.x)
    err_yaw = normalize_angle(desired_yaw - yaw2_)

    #rospy.loginfo(err_yaw)

    twist_msg2 = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg2.angular.z = 0.2 if err_yaw > 0 else -0.2

    pub2.publish(twist_msg2)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state2(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
        pub.publish(twist_msg)
    else:
        #print 'Position error: [%s]' % err_pos
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print 'Yaw error: [%s]' % err_yaw
        change_state(0)

def go_straight_ahead2(des_pos):
    global yaw2_, pub2, yaw_precision_, state2_
    desired_yaw = math.atan2(des_pos.y - position2_.y, des_pos.x - position2_.x)
    err_yaw = desired_yaw - yaw2_
    err_pos = math.sqrt(pow(des_pos.y - position2_.y, 2) + pow(des_pos.x - position2_.x, 2))

    if err_pos > dist_precision_:
        twist_msg2 = Twist()
        twist_msg2.linear.x = 0.6
        twist_msg2.angular.z = 0.1 if err_yaw > 0 else -0.1
        pub2.publish(twist_msg2)
    else:
        #print 'Position error: [%s]' % err_pos
        change_state2(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print 'Yaw error: [%s]' % err_yaw
        change_state2(0)

def done_point():
    global desired_position_,state_,state2_
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    if state_ == 2:
        pub.publish(twist_msg)
    if state2_ == 2:
        pub2.publish(twist_msg)

    if state_ == 2 and state2_ == 2:
        initial_position.x = desired_position_.x
        initial_position.y = desired_position_.y
        initial_position2.x = desired_position2_.x
        initial_position2.y = desired_position2_.y
        set_action__i()
    c = 0
    for i in range(len(explore)):
        for j in range(len(explore[i])):
            if(explore[i][j] > 0):
                c = c + 1
    if(c == grid_size*grid_sizey):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        pub2.publish(twist_msg)
        rospy.loginfo('Done')
        sys.exit()

def movement(agent,action):
    global desired_position_,desired_position2_,initial_position,initial_position2,direction,direction2
    flag = 0
    if agent == 1:
        c = 0
        while flag == 0:
            if action == 1:
                if direction == 1:
                    desired_position_.x = initial_position.x - 2
                    desired_position_.y = initial_position.y
                elif direction == 2:
                    desired_position_.x = initial_position.x
                    desired_position_.y = initial_position.y +2
                elif direction == 3:
                    desired_position_.x = initial_position.x + 2
                    desired_position_.y = initial_position.y
                elif direction == 4:
                    desired_position_.x = initial_position.x
                    desired_position_.y = initial_position.y - 2
                if ((desired_position_.x >=0 and desired_position_.x<=grid_size) and (desired_position_.y>=0 and desired_position_.y<=grid_sizey)):
                    flag = 1
                else:
                    action = 2

            if action == 2:
                if direction == 1:
                    desired_position_.x = initial_position.x
                    desired_position_.y = initial_position.y + 1
                elif direction == 2:
                    desired_position_.x = initial_position.x +1
                    desired_position_.y = initial_position.y
                elif direction == 3:
                    desired_position_.x = initial_position.x
                    desired_position_.y = initial_position.y - 1
                elif direction == 4:
                    desired_position_.x = initial_position.x - 1
                    desired_position_.y = initial_position.y

                if ((desired_position_.x >=0 and desired_position_.x<=grid_size) and (desired_position_.y>=0 and desired_position_.y<=grid_sizey)):
                    flag = 1
                    direction = direction + 1
                    if direction > 4:
                        direction = direction - 4
                else:
                    action = 3

            if action == 3:
                if direction == 1:
                    desired_position_.x = initial_position.x + 1
                    desired_position_.y = initial_position.y
                elif direction == 2:
                    desired_position_.x = initial_position.x
                    desired_position_.y = initial_position.y - 1
                elif direction == 3:
                    desired_position_.x = initial_position.x -1
                    desired_position_.y = initial_position.y
                elif direction == 4:
                    desired_position_.x = initial_position.x
                    desired_position_.y = initial_position.y + 1

                if ((desired_position_.x >=0 and desired_position_.x<=grid_size) and (desired_position_.y>=0 and desired_position_.y<=grid_sizey)):
                    flag = 1
                    direction = (direction + 2)
                    if direction > 4:
                        direction = direction - 4
                else:
                    action = 4
            if action == 4:
                if direction == 1:
                    desired_position_.x = initial_position.x
                    desired_position_.y = initial_position.y -1
                elif direction == 2:
                    desired_position_.x = initial_position.x - 1
                    desired_position_.y = initial_position.y
                elif direction == 3:
                    desired_position_.x = initial_position.x
                    desired_position_.y = initial_position.y + 1
                elif direction == 4:
                    desired_position_.x = initial_position.x + 1
                    desired_position_.y = initial_position.y

                if ((desired_position_.x >=0 and desired_position_.x<=grid_size) and (desired_position_.y>=0 and desired_position_.y<=grid_sizey)):
                    flag = 1
                    direction = (direction - 1)
                    if direction == 0:
                        direction = 4
                else:
                    action = 1
                    c = c+1
                if c>5:
                    for i in range(len(explore)):
                        for j in range(len(explore[i])):
                            if explore[i][j] == 0:
                                desired_position_.x = i
                                desired_position_.y = j
                    flag = 1



    if agent == 2:
        c = 0
        while flag == 0:
            if action == 1:
                if direction2 == 1:
                    desired_position2_.x = initial_position2.x - 2
                    desired_position2_.y = initial_position2.y
                elif direction2 == 2:
                    desired_position2_.x = initial_position2.x
                    desired_position2_.y = initial_position2.y +2
                elif direction2 == 3:
                    desired_position2_.x = initial_position2.x + 2
                    desired_position2_.y = initial_position2.y
                elif direction2 == 4:
                    desired_position2_.x = initial_position2.x
                    desired_position2_.y = initial_position2.y - 2
                if ((desired_position2_.x >=0 and desired_position2_.x<=grid_size) and (desired_position2_.y>=0 and desired_position2_.y<=grid_sizey)):
                    flag = 1
                else:
                    action = 2

            if action == 2:
                if direction2 == 1:
                    desired_position2_.x = initial_position2.x
                    desired_position2_.y = initial_position2.y + 1

                elif direction2 == 2:
                    desired_position2_.x = initial_position2.x +1
                    desired_position2_.y = initial_position2.y
                elif direction2 == 3:
                    desired_position2_.x = initial_position2.x
                    desired_position2_.y = initial_position2.y - 1
                elif direction == 4:
                    desired_position2_.x = initial_position2.x - 1
                    desired_position2_.y = initial_position2.y

                if ((desired_position2_.x >=0 and desired_position2_.x<=grid_size) and (desired_position2_.y>=0 and desired_position2_.y<=grid_sizey)):
                    flag = 1
                    direction2 = direction2 + 1
                    if direction2 > 4:
                        direction2 = direction2 - 4
                else:
                    action = 3

            if action == 3:
                if direction2 == 1:
                    desired_position2_.x = initial_position2.x + 1
                    desired_position2_.y = initial_position2.y
                elif direction2 == 2:
                    desired_position2_.x = initial_position2.x
                    desired_position2_.y = initial_position2.y - 1
                elif direction2 == 3:
                    desired_position2_.x = initial_position2.x -1
                    desired_position2_.y = initial_position2.y
                elif direction2 == 4:
                    desired_position2_.x = initial_position2.x
                    desired_position2_.y = initial_position2.y + 1

                if ((desired_position2_.x >=0 and desired_position2_.x<=grid_size) and (desired_position2_.y>=0 and desired_position2_.y<=grid_sizey)):
                    flag = 1
                    direction2 = (direction2 + 2)
                    if direction2 > 4:
                        direction2 = direction2 - 4
                else:
                    action = 4
            if action == 4:
                if direction2 == 1:
                    desired_position2_.x = initial_position2.x
                    desired_position2_.y = initial_position2.y -1
                elif direction2 == 2:
                    desired_position2_.x = initial_position2.x - 1
                    desired_position2_.y = initial_position2.y
                elif direction2 == 3:
                    desired_position2_.x = initial_position2.x
                    desired_position2_.y = initial_position2.y + 1
                elif direction2 == 4:
                    desired_position2_.x = initial_position2.x + 1
                    desired_position2_.y = initial_position2.y

                if ((desired_position2_.x >=0 and desired_position2_.x<=grid_size) and (desired_position2_.y>=0 and desired_position2_.y<=grid_sizey)):
                    flag = 1
                    direction2 = (direction2 - 1)
                    if direction2 == 0:
                        direction2 = 4
                else:
                    action = 1
                    c = c +1
                if c>5:
                    for i in range(len(explore)):
                        for j in range(len(explore[i])):
                            if explore[i][j] == 0:
                                desired_position_.x = i
                                desired_position_.y = j
                    flag = 1

def execute():
    global t,desired_position_,desired_position2_,initial_position,initial_position2
    if t == 0:
        action = 1
        desired_position_.x = initial_position.x + 2
        desired_position_.y = initial_position.y
        desired_position2_.x = initial_position2.x + 2
        desired_position2_.y = initial_position2.y
        change_state(0)
        change_state2(0)
        t = t+1
    else:
        print desired_position_
        print desired_position2_
        change_state(0)
        change_state2(0)

def set_action__i():
    global action
    agent = random.randint(1,2)
    flag = 0
    if agent == 1:
            movement(agent,action)
            set_all_explore(desired_position_,agent)
    elif agent == 2:
            movement(agent,action)
            set_all_explore(desired_position2_,agent)
    set_action_i(agent)

def set_action_i(agent):
    global direction,direction2,initial_position,initial_position2,desired_position_,desired_position2_
    utility = []
    borders = []

    if agent == 1:
            if direction2 == 1:
                temp = Point()
                temp.x = initial_position2.x - 2
                temp.y = initial_position2.y
                borders.append(temp)
                temp1 = Point()
                temp1.x = initial_position2.x
                temp1.y = initial_position2.y +1
                borders.append(temp1)
                temp2 = Point()
                temp2.x = initial_position2.x + 1
                temp2.y = initial_position2.y
                borders.append(temp2)
                temp3 = Point()
                temp3.x = initial_position2.x
                temp3.y = initial_position2.y -1
                borders.append(temp3)
                print temp
            elif direction2 == 2:
                temp = Point()
                temp.x = initial_position2.x
                temp.y = initial_position2.y + 2
                borders.append(temp)
                temp1 = Point()
                temp1.x = initial_position2.x +1
                temp1.y = initial_position2.y
                borders.append(temp1)
                temp2 = Point()
                temp2.x = initial_position2.x
                temp2.y = initial_position2.y - 1
                borders.append(temp2)
                temp3 = Point()
                temp3.x = initial_position2.x - 1
                temp3.y = initial_position2.y
                borders.append(temp3)
            elif direction2 == 3:
                temp = Point()
                temp.x = initial_position2.x + 2
                temp.y = initial_position2.y
                borders.append(temp)
                temp1 = Point()
                temp1.x = initial_position2.x
                temp1.y = initial_position2.y - 1
                borders.append(temp)
                temp2 = Point()
                temp2.x = initial_position2.x - 1
                temp2.y = initial_position2.y
                borders.append(temp)
                temp3 = Point()
                temp3.x = initial_position2.x
                temp3.y = initial_position2.y + 1
                borders.append(temp3)
            elif direction2 == 4:
                temp = Point()
                temp.x = initial_position2.x
                temp.y = initial_position2.y -2
                borders.append(temp)
                temp1 = Point()
                temp1.x = initial_position2.x -1
                temp1.y = initial_position2.y
                borders.append(temp1)
                temp2 = Point()
                temp2.x = initial_position2.x
                temp2.y = initial_position2.y + 1
                borders.append(temp2)
                temp3 = Point()
                temp3.x = initial_position2.x + 1
                temp3.y = initial_position2.y
                borders.append(temp3)

            print borders
            for i in range(len(borders)):
                utility.append(utility_calc(borders[i],2,i))
            for i in range (len(borders)):
                if borders[i].x <0 or borders[i].y <0 or borders[i].x >grid_size or borders[i].y > grid_sizey:
                    utility[i] = -99
            max = -1
            for i in range(len(utility)):
                if utility[i]> max:
                    max = i
            desired_position2_.x = borders[max].x
            desired_position2_.y = borders[max].y
            if desired_position_.x<0 or desired_position_.y<0 or desired_position_.x>grid_size or desired_position_.y>grid_sizey:
                    for i in range(len(explore)):
                        for j in range(len(explore[i])):
                            if explore[i][j] == 0:
                                desired_position_.x = i
                                desired_position_.y = j

            set_all_explore(desired_position2_,2)
            if max == 2:
                direction = direction + 1
                if direction >4:
                    direction = direction - 4
            if max == 3:
                direction = direction + 2
                if direction>4:
                    direction = direction -4
            if max == 4:
                direction = direction - 1
                if direction == 0:
                    direction = 4

    if agent == 2:
            if direction == 1:
                temp = Point()
                temp.x = initial_position.x - 2
                temp.y = initial_position.y
                borders.append(temp)
                temp1 = Point()
                temp1.x = initial_position.x
                temp1.y = initial_position.y +1
                borders.append(temp1)
                temp2 = Point()
                temp2.x = initial_position.x + 1
                temp2.y = initial_position.y
                borders.append(temp2)
                temp3 = Point()
                temp3.x = initial_position.x
                temp3.y = initial_position.y -1
                borders.append(temp3)
            elif direction == 2:
                temp = Point()
                temp.x = initial_position.x
                temp.y = initial_position.y + 2
                borders.append(temp)
                temp1 = Point()
                temp1.x = initial_position.x +1
                temp1.y = initial_position.y
                borders.append(temp1)
                temp2 = Point()
                temp2.x = initial_position.x
                temp2.y = initial_position.y - 1
                borders.append(temp2)
                temp3 = Point()
                temp3.x = initial_position.x - 1
                temp3.y = initial_position.y
                borders.append(temp3)
            elif direction == 3:
                temp = Point()
                temp.x = initial_position.x + 2
                temp.y = initial_position.y
                borders.append(temp)
                temp1 = Point()
                temp1.x = initial_position.x
                temp1.y = initial_position.y - 1
                borders.append(temp1)
                temp2 = Point()
                temp2.x = initial_position.x - 1
                temp2.y = initial_position.y
                borders.append(temp2)
                temp3 = Point()
                temp3.x = initial_position.x
                temp3.y = initial_position.y + 1
                borders.append(temp3)
            elif direction == 4:
                temp = Point()
                temp.x = initial_position.x
                temp.y = initial_position.y -2
                borders.append(temp)
                temp1 = Point()
                temp1.x = initial_position.x -1
                temp1.y = initial_position.y
                borders.append(temp1)
                temp2 = Point()
                temp2.x = initial_position.x
                temp2.y = initial_position.y + 1
                borders.append(temp2)
                temp3 = Point()
                temp3.x = initial_position.x + 1
                temp3.y = initial_position.y
                borders.append(temp3)
            for i in range(len(borders)):
                utility.append(utility_calc(borders[i],1,i))
            for i in range (len(borders)):
                if borders[i].x <0 or borders[i].y <0 or borders[i].x > grid_size or borders[i].y > grid_sizey:
                    utility[i] = -99
            max = -1
            for i in range(len(utility)):
                if utility[i]> max:
                    max = i
            desired_position_.x = borders[max].x
            desired_position_.y = borders[max].y
            if desired_position_.x<0 or desired_position_.y<0 or desired_position_.x>grid_size or desired_position_.y>grid_sizey:
                    for i in range(len(explore)):
                        for j in range(len(explore[i])):
                            if explore[i][j] == 0:
                                desired_position_.x = i
                                desired_position_.y = j
            set_all_explore(desired_position_,1)
            if max == 2:
                direction = direction + 1
                if direction >4:
                    direction = direction - 4
            if max == 3:
                direction = direction + 2
                if direction>4:
                    direction = direction -4
            if max == 4:
                direction = direction - 1
                if direction == 0:
                    direction = 4

    execute()

def utility_calc(des_position_,agent,j):
    global explore
    point = Point()
    u = 0
    if j == 0:
        for i in range(grid_size):
            if des_position_.x-i<grid_size and des_position_.x-i >=0 and des_position_.y<grid_sizey and des_position_.y>=0:
                print des_position_,i
                if explore[des_position_.x-i][des_position_.y] == 0:
                    u = u+1
            else:
                u = -1
    elif j == 1:
        for i in range(grid_sizey):
            if des_position_.y+i<grid_sizey and des_position_.y+i >=0 and des_position_.x<grid_size and des_position_.x>=0:
                print des_position_,i
                if explore[des_position_.x][des_position_.y+i] == 0:
                    u = u+1
            else:
                u = -1
    elif j == 2:
        for i in range(grid_size):
            if des_position_.x+i<grid_size and des_position_.x+i >=0 and des_position_.y<grid_size and des_position_.y>=0:
                print des_position_,i
                if explore[des_position_.x+i][des_position_.y] == 0:
                    u = u+1
            else:
                u = -1
    elif j ==3:
        for i in range(grid_sizey):
            if des_position_.y-i<grid_sizey and des_position_.y-i >=0 and des_position_.x<grid_size and des_position_.x>=0:
                print des_position_,i
                if explore[des_position_.x][des_position_.y-i] == 0:
                    u = u+1
            else:
                u = -1
    return u
def set_all_explore(curr_point,agent):
    global explore, range_
    for i in range(curr_point.x-range_,curr_point.x+range_):
        for j in range(curr_point.y-range_,curr_point.y+range_):
            if(((i-curr_point.x)*(i-curr_point.x) + (j-curr_point.y)*(j-curr_point.y) <= range_*range_) and (i<=grid_size-1 and i>=0 and j<=grid_sizey-1 and j>=0)):
                explore[i][j] = agent
            else:
                continue
    for i in range(len(explore)):
        for j in range(len(explore[i])):
            print explore[i][j],
        print

def main():
    global pub, active_,pub2
    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)


    sub_odom = rospy.Subscriber('/robot1/odom', Odometry, clbk_odom)
    sub_odom2 = rospy.Subscriber('/robot2/odom', Odometry, clbk_odom2)



    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
               done_point()
            else:
                rospy.logerr('Unknown state!')

            if state2_ == 0:
                fix_yaw2(desired_position2_)
            elif state2_ == 1:
                go_straight_ahead2(desired_position2_)
            elif state2_ == 2:
               done_point()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()

if __name__ == '__main__':
    main()
