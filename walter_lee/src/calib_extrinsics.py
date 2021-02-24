#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseArray, TransformStamped
from cv_bridge import CvBridge
import tf_conversions
from tf2_ros import StaticTransformBroadcaster

def PoseArray2List(poseA):
    '''Convert a PoseArray to an array of points, to a list of points in format [1x2]'''
    ptlist = []
    for pose in poseA.poses:
        ptlist.append( np.array([pose.position.x,pose.position.y]).reshape((1,2)) )        
    return ptlist

def draw_pts(im_in, pix_pts, labels=[]):
        
    for pt in pix_pts:
        cv2.drawMarker(im_in, (int(pt[0][0]), int(pt[0][1])), color=(0,200,255), markerSize=20, thickness=2)

    for i,label in enumerate(labels):
        xy = ( int(pix_pts[i][0][0])-20, int(pix_pts[i][0][1])-20 )
        cv2.putText(im_in, label, xy, cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,200,255),1)

    return im_in


class calib_extrinsics():
    def __init__(self):

        rospy.sleep(0.2)        

        im_msg = rospy.wait_for_message('/raspicam_node/image/compressed', CompressedImage)
        cam_msg = rospy.wait_for_message('/raspicam_node/camera_info', CameraInfo)
        pose_msg = rospy.wait_for_message('/dots', PoseArray)
        rospy.loginfo("Read in image + points")
        
        bridge = CvBridge()
        im_in = bridge.compressed_imgmsg_to_cv2(im_msg, "bgr8")

        pix_pts = PoseArray2List(pose_msg)

        im_out = draw_pts(im_in, pix_pts)
        cv2.imshow('image_in', im_out)
        cv2.waitKey(2000); 
        cv2.destroyAllWindows()
        
        K = np.array(cam_msg.K).copy().reshape((3,3))
        D = np.array(cam_msg.D).copy()

        #Convert to array of undistorted pixels on the unit focal plane:
        ud_pts = cv2.undistortPoints( np.array(pix_pts), K, D, P=None)    

        R, t = self.calc_extrinsics( ud_pts )

        rospy.loginfo('Rotation c2b: '+str(R))
        rospy.loginfo('Translation c2b: '+str(t))

        self.publish(R, t)
        

    def calc_extrinsics(self, ud_pts):
        '''inputs 2 points that define rays
        First ray is directly infront of robot at same height as camera
        second ray is to the right of the first on the same horizontal plane'''

        p_f = np.concatenate( (ud_pts[0,0,:], [1]) )  # shape: (3,)
        p_r = np.concatenate( (ud_pts[1,0,:], [1]) )  # shape: (3,)

        m_x = -p_f / np.linalg.norm( p_f ) # (3,) x axis of robot in camera coords
        m_z = np.cross(p_r, p_f)  
        m_z /= np.linalg.norm( m_z )       # (3,) z axis of robot in camera coords
        m_y = np.cross( m_z, m_x )         # (3,) y axis of robot in camera coords

        R_cam2base = np.stack( (m_x, m_y, m_z) )  # Stacking row vectors
        R44 = np.eye(4)                    # 4x4 is required for convertion to quaternions
        R44[:3,:3] = R_cam2base
        R_cam2base_q = tf_conversions.transformations.quaternion_from_matrix( R44 )  

        t_cam2base = np.array([-0.1, 0, 0.14])  # Position of camera in base_footprint
     
        return R_cam2base_q, t_cam2base

    def publish(self, R_q, t):
        '''Publish static transform for base_footprint to camera_pose
        This can be published via the command-line too'''
        broadcaster = StaticTransformBroadcaster()  # From tf2_ros
        static_tf = TransformStamped()              # A message from geometry_msgs.msg

        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "base_footprint"
        static_tf.child_frame_id = "camera_pose"

        static_tf.transform.translation.x = t[0]
        static_tf.transform.translation.y = t[1]
        static_tf.transform.translation.z = t[2]

        static_tf.transform.rotation.x = R_q[0]
        static_tf.transform.rotation.y = R_q[1]
        static_tf.transform.rotation.z = R_q[2]
        static_tf.transform.rotation.w = R_q[3]

        broadcaster.sendTransform(static_tf)        
        rospy.loginfo("Broadcasting static transform")

if __name__=='__main__':

    rospy.init_node('calib_extrinsics')

    calib_extrinsics()

    rospy.spin()

