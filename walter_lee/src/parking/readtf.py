#!/usr/bin/env python  
import rospy
import tf

class get_cam_tf:
    '''Initializing this class will read the transformation from base_footprint to camera_pose
    It is stored in self.trans and self.rot, and this happens in parallel to subsequent commands'''
    def __init__(self, doWait=True):
        self.listen = tf.TransformListener()
        self.timer = rospy.Timer( rospy.Duration(0.1), self.read )  #calls read() every 0.1 sec
        self.trans = []  #translation will be filled as list: [tx,ty,tz]
        self.rot = []    #rotation will be filled as quaternion list: [qx,qy,qz,qw]
        if doWait:
            self.wait()  #Wait until got transform before doing anything else

    def read(self, timer_event):
        timeout = rospy.Duration(0.9)
        try:            
            self.listen.waitForTransform("base_footprint", "camera_pose", rospy.Time.now(), timeout)
            self.trans, self.rot = self.listen.lookupTransform("base_footprint", "camera_pose", rospy.Time(0))
            rospy.loginfo('Obtained transformation base_footprint --> camera_pose')
            #Note: it is necessary to use Time(0) here, as this tells the listener to get the latest
            self.timer.shutdown()  # Done so stop timer -- can now use self.trans and self.rot
        except:
            pass
    def wait(self):
        rate = rospy.Rate(4)
        if not self.trans:
            rospy.loginfo('Waiting for base_footprint and camera_pose')
        while not self.trans:
            rate.sleep()

if __name__ == '__main__':    
    ''' Example node to read in transformation from base_footprint to camera_pose'''
    rospy.init_node('tf_reader')
    camtf = get_cam_tf(False)
    rospy.loginfo('Trans: ' + str(camtf.trans) + ', Rot: ' + str(camtf.rot) )
    rospy.sleep(1.)  #Note: it takes a little while to initialize the transforms, as illustrated here
    rospy.loginfo('Trans: ' + str(camtf.trans) + ', Rot: ' + str(camtf.rot) )
    rospy.sleep(1.)
