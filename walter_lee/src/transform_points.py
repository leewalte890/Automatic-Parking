#!/usr/bin/env python
'''Convert points held in PoseArray from one frame to another frame
See example of use in estimate_floor_points'''
from __future__ import print_function
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Header
import tf2_ros, tf2_geometry_msgs

class trans_frames():
    def __init__(self):
        '''Will maintain a buffer of transforms'''
        self.tfBuffer = tf2_ros.Buffer()
        listen = tf2_ros.TransformListener(self.tfBuffer)
    
    def get_trans(self, from_frame,to_frame):
        try:
            trans = self.tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(0.2) )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Error, cannot find transformation from %s to %s' % from_frame, to_frame)
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



