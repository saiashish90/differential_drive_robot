import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math
import random

#range_
range_ = 3

#explore grid size
explore = [[0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0]]
desired_position_ = Point()

def set_all_points_one(curr_point):
    global explore, range_
    for i in range(curr_point.x-range_,curr_point.x+range_):
        for j in range(curr_point.y-range_,curr_point.y+range_):
            if(((i-curr_point.x)*(i-curr_point.x) + (j-curr_point.y)*(j-curr_point.y)) <= range_*range_):
                if(i>=0 and j>=0):
                    explore[i][j] = 1

            else:
                continue
def set_point_one(point):
    global explore
    explore[point.x][point.y] = 1

def main():
    global desired_position_
    desired_position_.x = 1
    desired_position_.y = 1
    set_point_one(desired_position_)
    set_all_points_one(desired_position_)
    for i in range(0,10):
        for j in range(0,10):
            print i,j,
            print ':',
            print(explore[i][j])
        print
    print explore[0][0]

if __name__ == '__main__':
    main()
