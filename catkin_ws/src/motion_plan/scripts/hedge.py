import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math
import random

#range_
range_ = 2
grid_size = 10

#explore grid size
explore = []
for i in range(grid_size):
    temp = []
    for j in range(20):
        temp.append(0)
    explore.append(temp)

circle = []
for i in range(int(math.ceil(float(grid_size)/(2*range_)))):
    temp = []
    for j in range(int(math.ceil(float(grid_size)/(2*range_)))):
        a = Point()
        temp.append(a)
    circle.append(temp)

circle[0][0].x = 1
circle[0][0].y = 2
for i in range(len(circle)):
    for j in range(len(circle[i])):
        if i == 0 and j == 0:
            circle[i][j].x = 0
            circle[i][j].y = 0
        elif i%2 == 0:
            if j == 0:
                circle[i][j].x = 0
                circle[i][j].y = 2*i*range_
            else:
                circle[i][j].x = circle[i][j-1].x+2*range_
                circle[i][j].y = 2*i*range_
        else:
            if j == 0:
                circle[i][j].x = 0+range_
                circle[i][j].y = 2*i*range_
            else:
                circle[i][j].x = circle[i][j-1].x+2*range_
                circle[i][j].y = 2*i*range_

def main():
    borders = []
    global explore, circle
    for i in range(len(circle)):
        for j in range(len(circle[i])):
            print circle[i][j].x,circle[i][j].y,
        print
    if():
        print borders
if __name__ == '__main__':
    main()
