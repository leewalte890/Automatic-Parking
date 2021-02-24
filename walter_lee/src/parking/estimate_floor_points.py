#!/usr/bin/env python
''' This Node reads in points assumed to be on the floor from /dots, and estimates their position relative
to base_footprint, and then transforms them to odom coordinates'''
from __future__ import print_function
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from cv_bridge import CvBridge
import tf_conversions
from readtf import get_cam_tf
from calib_extrinsics import PoseArray2List
from transform_points import trans_frames

class dot_estimator():
    def __init__(self, R_cam2base_q, t_cam2base):

        rospy.sleep(0.2)        

        im_msg = rospy.wait_for_message('/raspicam_node/image/compressed', CompressedImage)
        cam_msg = rospy.wait_for_message('/raspicam_node/camera_info', CameraInfo)
        pose_msg = rospy.wait_for_message('/dots', PoseArray)
        trans = trans_frames()    # For transforming between frames
        rospy.loginfo("Read in image + points")
        
        bridge = CvBridge()
        im_in = bridge.compressed_imgmsg_to_cv2(im_msg, "bgr8")

        pix_pts = PoseArray2List(pose_msg)

        K = np.array(cam_msg.K).copy().reshape((3,3))
        D = np.array(cam_msg.D).copy()

        #Convert to array of undistorted pixels on the unit focal plane:
        ud_pts = cv2.undistortPoints( np.array(pix_pts), K, D, P=None)    

        rospy.loginfo('Calculating floor points in base_footprint:')
        pts_base = self.floorPoints( R_cam2base_q, t_cam2base, ud_pts )
        for i, pt in enumerate(pts_base.poses):
            rospy.loginfo('%d: %7.4f %7.4f %7.4f' % (i, pt.position.x, pt.position.y, pt.position.z ) )

        #Now transform these points to world (or odom) coordinates:
        #This assumes tf is being published (running turtlebot or gazebo will do this)
        rospy.loginfo('Transflorming floor points to odom:')
        pts_odom = trans.pose_trans(pts_base, 'base_footprint', 'odom')
        for i, pt in enumerate(pts_odom.poses):
            rospy.loginfo('%d: %7.4f %7.4f %7.4f' % (i, pt.position.x, pt.position.y, pt.position.z ) )

    def floorPoints(self, R_cam2base_q, t_cam2base, ud_pts):
        ''' Given transformation of camera points to base points, and a set of undistorted pixel rays
        find the 3D location that these rays intersect the ground in base_footprint coordinates'''
        if not t_cam2base:
            rospy.logerr("Empty translation provided")
        R = tf_conversions.transformations.quaternion_matrix( R_cam2base_q)
        pts_base = PoseArray()
        N = ud_pts.shape[0]
        udh_pts = np.concatenate( (ud_pts, np.ones((N,1,1))), axis=2 )  #make homogeneous
        for i in np.arange(N):
            v = np.dot( R[:3,:3], udh_pts[i,:,:].T )
            lam = -t_cam2base[2]/v[2]
            pt = np.array(t_cam2base)[:,None] + lam * v
            pose_T = Pose()
            pose_T.position.x, pose_T.position.y, pose_T.position.z = pt[0], pt[1], pt[2] 
            pts_base.poses.append(pose_T)
        
        return pts_base

if __name__=='__main__':

    rospy.init_node('ex_calibration')

    camtf = get_cam_tf()  # Get camera transform

    de = dot_estimator( camtf.rot, camtf.trans)

    rospy.spin()

