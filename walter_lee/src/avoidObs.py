#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseArray, PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from tf.transformations import quaternion_from_euler, quaternion_multiply
from sensor_msgs import point_cloud2 as pc2
from nav_msgs.msg import Odometry
import tf2_ros, tf2_geometry_msgs
import copy


class search():
    def __init__(self):
        rospy.init_node('avoidObs')
        self.y0 = 0
        self.x0 = 0
        self.waypoint = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move = Twist()
        self.tfBuffer = tf2_ros.Buffer()
        listen = tf2_ros.TransformListener(self.tfBuffer)
        self.way_pub = rospy.Publisher('waypoints', PoseArray, queue_size=1)
        self.subp = rospy.Subscriber('/scan_PointCloud2', PointCloud2, self.pc_callback)
        self.subo = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pubVel = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
        rospy.Subscriber("/points", PoseArray, self.points)
        rospy.spin()

    def points(self, msg):
        x = msg.poses[0].position.x
        y = msg.poses[0].position.y

        # theta = math.atan2(y-self.y0, x-self.x0)
        # xD = math.cos(theta)
        # yD = math.sin(theta)
        # self.direction = (xD,yD)
        # self.y0 = y
        # self.x0 = x
        # print(x,y)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def pc_callback(self, msg):
        '''look for object >= 0.5m away'''
        try:
            if self.wait:
                return
        except AttributeError:
            self.wait = False
        
        '''List consisting of xyz coords from /scan_PointCloud2 topic'''
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        '''x and y values from point in front of robot'''
        x = cloud_points[0][0]
        y = cloud_points[0][1]
        #print(math.sqrt(x**2+y**2))

        if math.sqrt(x**2+y**2) <= 0.6:
            if self.waypoint == 0:
            # move = Twist()
            # move.linear.x = 0
            # move.angular.z = 0
            # self.pubVel.publish(move)
                self.waypoint = 1
                self.pubMove()

    def get_trans(self, from_frame,to_frame):
        try:
            trans = self.tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(0.2) )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Error, cannot find transformation from %s to %s' % (from_frame, to_frame))
        return trans

    def pose_trans(self, poseA, from_frame='base_footprint', to_frame='odom'):
        '''Input a PoseArray containing points in from_frame, and return a PoseArray with 
        the same points in to_frame
        If cannot find transform, returns empty PoseArray'''
        trans = self.get_trans(from_frame,to_frame)
        poseT = PoseArray()  # Transformed points
        header = Header(frame_id=from_frame, stamp=rospy.Time(0))
        for pose in poseA.poses:
            pose_s = PoseStamped(pose=pose, header=header)
            pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
            poseT.poses.append( pose_t.pose )
        return poseT
                
    def pubMove(self):
        self.wait = True
        rospy.loginfo('Object detected, exploring')

        palist = PoseArray()
        pnewlist = PoseArray()

        palist.poses.append(self.pose)
        baseArray = self.pose_trans(palist,'odom','base_footprint')
        #mapArray = self.pose_trans(palist,'odom','map')

        p01 = baseArray.poses[0]
        p00 = copy.deepcopy(p01)

        p01.position.x += 1
        p00.position.y -= 0.5
        pnewlist.poses.append(p00)
        pnewlist.poses.append(p01)
        newodom = self.pose_trans(pnewlist,'base_footprint', 'odom')
        newmap = self.pose_trans(newodom,'odom','map')

        p1 = newmap.poses[1]
        #p0 = copy.deepcopy(p1)
        # ori1 = copy.deepcopy(p1.orientation)
        # oriNew = quaternion_from_euler(0, 0, -math.pi/2)
        # p1.orientation = quaternion_multiply(oriNew,ori1)
        p1.orientation.x = 0
        p1.orientation.y = 0
        p1.orientation.z = 0
        p1.orientation.w = 1

        #if abs(self.direction[0]) > abs(self.direction[1]):
            #p0.position.y += self.direction[1]
            #p1.position.x += self.direction[0]
        #else:
            #p0.position.x += self.direction[0]
            #p1.position.y += self.direction[1]
        
        #p1.position.y += 0.7

        #fList = PoseArray()
        #fList.poses.append(p0)
        #fList.poses.append(p1)
        
        rospy.sleep(1)
        #self.way_pub.publish(fList)
        #rospy.loginfo('Publishing '+str(len(palist.poses))+' waypoints'
        self.way_pub.publish(newmap)

        

def main():
    
    search()

if __name__ == '__main__':
    main()