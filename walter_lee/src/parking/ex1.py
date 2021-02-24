#!/usr/bin/env python

import os
import argparse
import rospy
import cv2
import numpy
import tf
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LogisticRegression
from readtf import get_cam_tf
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class LogRegression:
    def __init__(self, img, mask):
        self.img = img #Original parking space image
        self.mask = mask #Mask of parking space
        self.bridge = CvBridge()
        self.current = []
        self.cenCenter = []
        
        self.pos_pub = rospy.Publisher('/points', PoseArray, latch=True, queue_size=1)
        grayMask = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
        (thresh,binMask) = cv2.threshold(grayMask, 127, 255, cv2.THRESH_BINARY)

        self.logr = LogisticRegression(class_weight = 'balanced', solver='lbfgs')
        data = img.reshape((-1,3)).astype(float)
        self.label = (binMask.ravel()>0).astype(int)
        self.logr.fit(data,self.label)

        rospy.Subscriber('/camera/image/compressed', CompressedImage, self.display_compressed)
    
    def display_compressed(self, data):
        try:
            self.current = self.bridge.compressed_imgmsg_to_cv2(data,'bgr8')
        except CvBridgeError as e:
            print(e)
        
        probImg = self.regression()
        fProb = numpy.array(probImg.copy(),dtype = numpy.uint8)

        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(fProb)
        lAreas = stats[1:,cv2.CC_STAT_AREA]
        nAreas = list(lAreas)
        nCentroids = centroids[1:]
        try:
            cIndex = nAreas.index(max(nAreas))
        except ValueError:
            pass
        if len(nCentroids) > 0:
            cv2.circle(fProb,(int(nCentroids[cIndex][0]),int(nCentroids[cIndex][1])),5,(0,0,0),-1)
            self.cenCenter = [int(nCentroids[cIndex][0]),int(nCentroids[cIndex][1])]

        pArray = PoseArray()
        p0 = Pose()
        p0.position.x = self.cenCenter[0]
        p0.position.y = self.cenCenter[1]
        p0.position.z = 0
        p0.orientation.w = 1
        pArray.poses.append(p0)
        #print(self.cenCenter[0],self.cenCenter[1])

        pix_pts = PoseArray2List(pArray)

        K = np.array(self.cam_msg.K).copy().reshape((3,3))
        D = np.array(self.cam_msg.D).copy()

        camtf = get_cam_tf()
        
        ud_pts = cv2.undistortPoints( np.array(pix_pts, dtype=np.float32), K, D, P=None)
        pts_base = self.floorPoints( camtf.rot, camtf.trans, ud_pts )
        #pts_odom = trans.pose_trans(pts_base, 'base_footprint', 'odom')
        #print(pts_base.poses[0])
        if not self.cenCenter:
            nArray = PoseArray()
            p1 = Pose()
            p1.position.x = 0
            p1.position.y = 0
            p1.position.z = -1
            p1.orientation.w = 1
            nArray.poses.append(p0)
            self.pos_pub.publish(nArray)
        else:
            self.pos_pub.publish(pts_base)

        # cv2.imshow("Original Image", self.current.copy())
        # cv2.imshow("Probability", fProb)
        # cv2.waitKey(1)

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
        
        # cv2.imshow("Original Image", self.current.copy())
        # cv2.imshow("Probability", fProb)
        # cv2.waitKey(1)
    
    def regression(self):
        
        if len(self.current) > 0:
            vData = numpy.reshape(self.current,(-1,3)).astype(float)
            pixProb = self.logr.predict_proba(vData)
            probImg = []
            for x in pixProb:
                if x[1] > 0.9:
                    probImg.append(255)
                else:
                    probImg.append(0)
            probImg = numpy.reshape(probImg,(240,320))

            return probImg
        else:
            return 0

    
            
        
if __name__ == "__main__":

    # parser = argparse.ArgumentParser(description='Image and Mask')
    # parser.add_argument('imgName', type=str, metavar='PATH', help='Full path of image name')
    # parser.add_argument('maskName', type=str, metavar='PATH', help='Full path of mask name')

    # args = parser.parse_args()

    # if os.path.isfile(args.imgName):
    #     print("Reading in image:", args.imgName)
    #     img = cv2.imread(args.imgName)
    # else:
    #     print("Error: unable to find:", args.imgName)

    # if os.path.isfile(args.maskName):
    #     print("Reading in image:", args.maskName)
    #     mask = cv2.imread(args.maskName)
    # else:
    #     print("Error: unable to find:", args.maskName)
    
    rospy.init_node('parking', anonymous=True)

    path1 = r'/home/robot/catkin_ws/src/walter_lee/src/spot.png'
    path2 = r'/home/robot/catkin_ws/src/walter_lee/src/spot_mask.png'
    lr = LogRegression(img, mask)
    lr.regression()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()