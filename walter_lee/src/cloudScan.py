#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class test_node():
    def __init__(self):
        rospy.init_node('test')
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)  
        self.pts_pub = rospy.Publisher('/scan_PointCloud2', PointCloud2, queue_size=1)     
        rospy.spin()

    def lidar_callback(self, msg):      
        msg_PointCloud2 = LaserScan2PointCloud2(msg)   
        self.pts_pub.publish(msg_PointCloud2)


def LaserScan2PointCloud2(msg):
    pts = []
    for i, r in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        intensity = msg.intensities[i]
        pts.append([x, y, 0, intensity])

    cluster = []
    clusterList = []
    cluster.append(pts[0])
    for i in range(len(pts)-1):
        dx = abs(pts[i][0] - pts[i+1][0])
        dy = abs(pts[i][1] - pts[i+1][1])
        if dx < 0.5 and dy < 0.5:
            cluster.append(pts[i+1])
        else:
            newCluster = []
            newCluster.append(pts[i+1])
            clusterList.append(cluster)
            cluster = newCluster
    if clusterList[-1] != cluster:
        clusterList.append(cluster)

    xAprx = np.isclose(clusterList[0][0][0],clusterList[-1][-1][0],rtol=0.1)
    yAprx = np.isclose(clusterList[0][0][1],clusterList[-1][-1][1],rtol=0.1)
    if xAprx and yAprx:
        newCluster = []
        for i in range(len(clusterList[-1])-1):
            clusterList[0].append(clusterList[-1][i])
        clusterList[0] = newCluster
        clusterList.pop()

    cenList = []
    for cluster in clusterList:
        length = len(cluster)
        cluster = np.array(cluster)
        if length > 0:
            sumX = np.sum(cluster[:,0])
            sumY = np.sum(cluster[:,1])
            avg = (sumX/length,sumY/length)
            cenList.append(avg)

    newpts = []
    for centroid in cenList:
        newpts.append([centroid[0],centroid[1],0,10])

    fields = [PointField('x', 0, PointField.FLOAT32,1),
                PointField('y', 4, PointField.FLOAT32,1),
                PointField('z', 8, PointField.FLOAT32,1),
                PointField('intensity', 12, PointField.UINT32,1)]
    msg_PointCloud2 = point_cloud2.create_cloud(msg.header, fields, newpts)

    return msg_PointCloud2      
    
if __name__ == '__main__':
    test_node()
