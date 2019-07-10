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
grid_size = 10

#explore grid size
explore = []
for i in range(grid_size-1):
    temp = []
    for j in range(grid_size-1):
        temp.append(0)
    explore.append(temp)
for i in range(len(explore)):
    for j in range(len(explore[i])):
        print(explore[i][j]),
    print
explore[3][4] =25

def main():
    global explore
    print
    for i in range(len(explore)):
        for j in range(len(explore[i])):
            print(explore[i][j]),
        print
if __name__ == '__main__':
    main()
