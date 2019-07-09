import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math
import random

#range_
range_ = 1

#explore grid size
explore = [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
#for i in range(len(explore)):
    #for j in range(len(explore[i])):
        #print(explore[i][j]),
    #print

borders = []
borders.append(Point())
borders.append(Point())
borders.append(Point())
borders.append(Point())

desired_position_ = Point()
desired_position_.z = 0

def find_borders(des_position_):
    global explore, borders, desired_position_
    point = Point()
    point.z = 0

    #point1
    point.x =des_position_.x-1
    point.y =des_position_.y
    if(point.x>=0 and point.y>=0):
        borders[0].x = point.x
        borders[0].y = point.y
    else:
        borders[0].x = -1
        borders[0].y = -1

    #point2
    point.x =des_position_.x
    point.y =des_position_.y+1
    if(point.x>=0 and point.y>=0):
        borders[1].x = point.x
        borders[1].y = point.y

    else:
        borders[1].x = -1
        borders[1].y = -1

    #point3
    point.x =des_position_.x+1
    point.y =des_position_.y
    if(point.x>=0 and point.y>=0):
        borders[2].x = point.x
        borders[2].y = point.y
    else:
        borders[2].x = -1
        borders[2].y = -1

    #point4
    point.x =des_position_.x
    point.y =des_position_.y-1
    if(point.x>=0 and point.y>=0):
        borders[3].x = point.x
        borders[3].y = point.y
    else:
        borders[3].x = -1
        borders[3].y = -1

    #for i in range(0,len(borders)):
        #print (borders[i].x),
        #print (borders[i].y)

def pick_random_point():
    global borders, des_position_, explore
    flag = 0
    while(flag == 0):
        a = random.choice([0,1,2,3])
        if(borders[a].x>=0 and borders[a].y>=0 and explore[borders[a].x][borders[a].y] == 0):
            flag = 1
    desired_position_.x = borders[a].x
    desired_position_.y = borders[a].y
    print desired_position_
    for i in range(len(borders)):
        set_point_one(borders[i])
    for i in range(len(explore)):
        for j in range(len(explore[i])):
            print(explore[i][j]),
        print

def set_point_one(point):
    global explore
    explore[point.x][point.y] = 1

def main():
    global desired_position_
    desired_position_.x = 1
    desired_position_.y = 1
    set_point_one(desired_position_)
    find_borders(desired_position_)
    pick_random_point()
    set_point_one(desired_position_)
    find_borders(desired_position_)
    pick_random_point()

if __name__ == '__main__':
    main()
