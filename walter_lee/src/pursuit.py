#!/usr/bin/env python

import numpy as np
import rospy
import time
import math
import argparse
from copy import deepcopy
import copy as copy_module
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import Header, Bool
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import tf2_ros, tf2_geometry_msgs
import matplotlib.pyplot as plt

class pushFoward():
    def __init__(self):
        self.speed = 0.2
        self.x = 0
        self.y = 0
        self.go = 0
        self.reached = 0
        self.pubVel = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
        rospy.Subscriber("/points", PoseArray, self.points)
        rospy.Subscriber("/dots", PoseArray, self.dots)
        rospy.Subscriber("/waypointend", Bool, self.waypoint)
        rospy.Subscriber('/scan_PointCloud2', PointCloud2, self.pc_callback)
        #rospy.sleep(1)

    def waypoint(self, msg):
        if msg.data:
            self.reached = 1

    def pc_callback(self,msg):
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        '''x and y values from point in front of robot'''
        x = cloud_points[0][0]
        y = cloud_points[0][1]
        #print(math.sqrt(x**2+y**2))

        if math.sqrt(x**2+y**2) <= 0.6:
            self.go = 1
            move = Twist()
            move.linear.x = 0
            move.angular.z = 0
            self.pubVel.publish(move)

    def points(self,msg):
        self.x = msg.poses[0].position.x
        self.y = msg.poses[0].position.y
        
        if self.x == 0 and self.y == 0:
            self.speed = 0


        k = 2*self.y/(self.x**2 + self.y**2)
        if abs(k) > 5:
            k = 5*k/abs(k)

        if self.go == 0:
            move = Twist()
            move.linear.x = self.speed
            move.angular.z = (k*self.speed)
            self.pubVel.publish(move)

    def dots(self,msg):
        self.x = msg.poses[0].position.x
        self.y = msg.poses[0].position.y
        #print(self.x,self.y)

        if self.x == 0 and self.y == 0:
            self.speed = 0
            k = 0
        else:
            self.speed = 0.2
            k = 2*self.y/(self.x**2 + self.y**2)
            if abs(k) > 5:
                k = 5*k/abs(k)

        if self.reached == 1:
            move = Twist()
            move.linear.x = self.speed
            move.angular.z = (k*self.speed)
            self.pubVel.publish(move)

    # def arcCalc(self):
    #     x, y = self.x, self.y
    #     print(x, y)
    #     k = 2*y/(x**2 + y**2)
    #     if abs(k) > 5:
    #         k = 5*k/abs(k)
    #     return k

    # def angSpeed(self,k):
    #     move = Twist()
    #     move.linear.x = self.speed
    #     move.angular.z = (k*self.speed)
    #     self.pubVel.publish(move)
    #     #rospy.sleep(0.1)

if __name__=='__main__':
    rospy.init_node('pursuit')

    push = pushFoward()

    rospy.spin()