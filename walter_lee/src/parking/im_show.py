#!/usr/bin/env python
'''
Simple image viewer for raw or compressed video.  Usage:
 rosrun package im_show.py                    : this shows /raspicam_not/image/compressed  
 rosrun package im_show.py --raw              : this shows /camera/image_raw

Daniel Morris, Jan 2020
'''
import os
import argparse
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class im_show:
    
    def __init__(self, israw):
        self.bridge = CvBridge()
        self.current = []
        self.img_count = 0
        if israw:
            rospy.Subscriber('/camera/image_raw', Image, self.display_raw)
            rospy.loginfo('Initialized raw video viewer')
        else:
            rospy.Subscriber('/camera/image/compressed', CompressedImage, self.display_compressed)
            rospy.loginfo('Initialized compressed video viewer')

    def display_compressed(self, data):
        try:
            self.current = self.bridge.compressed_imgmsg_to_cv2(data,'bgr8')
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Compressed Video", self.current.copy())
        key = cv2.waitKey(1)
            
        #If space is hit, then it will enter if statement
        if key%256 == 32:
            filename = "ex1Image_{}.png".format(self.img_count)
            #directory = r'/home/robot/catkin_ws/src/walter_lee/src'
            cv2.imwrite(filename,self.current.copy())
            self.img_count+=1

    def display_raw(self, data):
        try:
            self.current = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Raw Video", self.current.copy())
        cv2.waitKey(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='image viewer')
    parser.add_argument('--raw', dest='raw', action='store_true', help='Set to view raw images')
    args = parser.parse_args()

    rospy.init_node('im_show', anonymous=True)
    ic = im_show( args.raw )
    print("Hit space to save current image frame")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

