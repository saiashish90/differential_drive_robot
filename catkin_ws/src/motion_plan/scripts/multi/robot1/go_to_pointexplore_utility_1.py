#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math
import random

#range_
range_ = 5
grid_size = 10

#explore grid size
explore = []
for i in range(grid_size):
    temp = []
    for j in range(grid_size):
        temp.append(0)
    explore.append(temp)

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0

prev = Point()

borders = []
borders.append(Point())
borders.append(Point())
borders.append(Point())
borders.append(Point())


# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
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

def find_borders(des_position_):
    global explore, borders, desired_position_, range_
    point = Point()
    point.z = 0

    #point1
    point.x =des_position_.x-range_
    point.y =des_position_.y
    if((point.x>=0 and point.y>=0)and (point.x<=grid_size-1 and point.y<=grid_size-1)):
        borders[0].x = point.x
        borders[0].y = point.y
    else:
        borders[0].x = -1
        borders[0].y = -1

    #point2
    point.x =des_position_.x
    point.y =des_position_.y+range_
    if((point.x>=0 and point.y>=0)and (point.x<=grid_size-1 and point.y<=grid_size-1)):
        borders[1].x = point.x
        borders[1].y = point.y

    else:
        borders[1].x = -1
        borders[1].y = -1

    #point3
    point.x =des_position_.x+range_
    point.y =des_position_.y
    if((point.x>=0 and point.y>=0)and (point.x<=grid_size-1 and point.y<=grid_size-1)):
        borders[2].x = point.x
        borders[2].y = point.y
    else:
        borders[2].x = -1
        borders[2].y = -1

    #point4
    point.x =des_position_.x
    point.y =des_position_.y-range_
    if((point.x>=0 and point.y>=0)and (point.x<=grid_size-1 and point.y<=grid_size-1)):
        borders[3].x = point.x
        borders[3].y = point.y
    else:
        borders[3].x = -1
        borders[3].y = -1

    #for i in range(0,len(borders)):
        #print (borders[i].x),
        #print (borders[i].y)
def utility_calc(des_position_):
    point = Point()
    u = 0
    point.x =des_position_.x-range_
    point.y =des_position_.y
    if((point.x>=0 and point.y>=0) and (point.x<=grid_size-1 and point.y<=grid_size-1) and explore[point.x][point.y] == 0 ):
        u= u+1
    point.x =des_position_.x
    point.y =des_position_.y+range_
    if((point.x>=0 and point.y>=0) and (point.x<=grid_size-1 and point.y<=grid_size-1) and explore[point.x][point.y] == 0 ):
        u= u+1
    point.x =des_position_.x+range_
    point.y =des_position_.y
    if((point.x>=0 and point.y>=0) and (point.x<=grid_size-1 and point.y<=grid_size-1) and explore[point.x][point.y] == 0 ):
        u= u+1
    point.x =des_position_.x
    point.y =des_position_.y-range_
    if((point.x>=0 and point.y>=0) and (point.x<=grid_size-1 and point.y<=grid_size-1) and explore[point.x][point.y] == 0 ):
        u= u+1
    return u

def find_max(util):
    max = -1
    for i in range(len(util)):
        if(util[i]> max ):
            max = i
    return max

def pick_random_point():
    global borders, des_position_, explore
    curr_point = Point()
    curr_point.x = desired_position_.x
    curr_point.y = desired_position_.y
    flag = 0
    utility = []
    max = -1
    pos = -1
    for i in range(len(borders)):
        utility.append(utility_calc(borders[i]))
        if(borders[i].x == -1 ):
            utility[i] = -1

    print utility
    while(flag != -1):
        #a = random.choice([0,1,2,3])
        #if((borders[a].x>=0 and borders[a].y>=0) and explore[borders[a].x][borders[a].y] == 0):
            #desired_position_.x = borders[a].x
            #desired_position_.y = borders[a].y
            #flag = -1

        max = find_max(utility)
        if(utility[max] > 0):
            desired_position_.x = borders[max].x
            desired_position_.y = borders[max].y
            flag = -1
        else:
            for i in range(len(explore)):
                for j in range(len(explore[i])):
                    if(explore[i][j] == 0):
                        if(range_>=1 and range_<=3):
                            desired_position_.x = i-(range_-1-1)
                            desired_position_.y = j-(range_-1-1)
                        elif(range_>=4 and range_<=7):
                            desired_position_.x = i-(range_-2-1)
                            desired_position_.y = j-(range_-2-1)
                        elif(range_>=8 and range_<=10):
                            desired_position_.x = i-(range_-3-1)
                            desired_position_.y = j-(range_-3-1)
                        #else:
                            #desired_position_.x = i
                            #desired_position_.y = j

                        if(desired_position_.x<0):
                            desired_position_.x = 0
                        if(desired_position_.y<0):
                            desired_position_.y = 0

            flag = -1

    print desired_position_

    set_all_points_one(curr_point)

    for i in range(len(explore)):
        for j in range(len(explore[i])):
            print(explore[i][j]),
        print

def set_point_one(point):
    global explore
    explore[point.x][point.y] = 1

def set_all_points_one(curr_point):
    global explore, range_
    for i in range(curr_point.x-range_,curr_point.x+range_):
        for j in range(curr_point.y-range_,curr_point.y+range_):
            if(((i-curr_point.x)*(i-curr_point.x) + (j-curr_point.y)*(j-curr_point.y) <= range_*range_) and (i<=grid_size-1 and i>=0 and j<=grid_size-1 and j>=0)):
                explore[i][j] = 1
            else:
                continue

def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

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

def done_point():
    global desired_position_
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    do_a_spin()
    set_point_one(desired_position_)
    find_borders(desired_position_)
    pick_random_point()
    change_state(0)

def do_a_spin():
    for i in range(200000):
        twist_msg = Twist()
        twist_msg.angular.z = 1
        pub.publish(twist_msg)

def main():
    global pub, active_
    c = 0

    rospy.init_node('go_to_point_1')

    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/robot1/odom', Odometry, clbk_odom)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

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
            c = 0
            for i in range(len(explore)):
                for j in range(len(explore[i])):
                    if(explore[i][j] == 1):
                        c = c + 1
            if(c == grid_size**2):
                twist_msg = Twist()
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                pub.publish(twist_msg)
                rospy.loginfo('Done')
                break


        rate.sleep()

if __name__ == '__main__':
    main()
