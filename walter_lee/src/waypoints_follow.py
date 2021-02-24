#!/usr/bin/env python
'''
  Waypoint follower.  Waypoints are assumed to be published as a PoseArray in 
  topic: /waypoints

  To start following waypoints, you can do the following:
  First start the robot.  For real Turtlebot, on the robot do:
   start_robot
  OR, using a simulated world, do:
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
  Then run SLAM:
   roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  Start the planner:
   roslaunch turtlebot3_navigation move_base.launch

  Create a ros package called <pkgname> and put .py files in its src folder. 
  Publish waypoints:
   rosrun <pkgname> waypoints_pub.py
  Follow waypoints:
   rosrun <pkgname> waypoints_follow.py

  Created by: Daniel Morris, March 2020
'''
__author__ = 'Daniel Morris'
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool


class waypoints_follow():

    def __init__(self):

        self.palist = []
        self.current = 0
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waypoint follower waiting for move_base action server...")
        # wait = self.client.wait_for_server(rospy.Duration(5.0))
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        #     return
        rospy.loginfo("Connected to move base server")
        #Now set up subscriber to waypoints.  Whenever waypoints topic is published
        #then follow them:
        rospy.Subscriber("waypoints", PoseArray, self.follow_points_cb )
        self.pub = rospy.Publisher('/waypointend', Bool, queue_size=1)

    def follow_points_cb(self, palist ):
        #When waypoints are published, this reads them can calls action server to follow them
        self.palist = palist
        self.current = 0
        self.start_at_first_waypoint()

    def start_at_first_waypoint(self):
        '''Select first waypoint and move to it.  
        When reached, move onto next (see self.done_waypoint_cb)'''
        first_waypoint = self.get_current_waypoint()
        if first_waypoint:
            rospy.loginfo("Sending waypoint "+str(self.current)+" to Action Server")
            rospy.loginfo(str(first_waypoint.target_pose))
            self.client.send_goal(first_waypoint, self.done_waypoint_cb, self.active_cb, self.feedback_cb)
        else:
            rospy.loginfo("No waypoints received")

    def get_current_waypoint(self):
        '''Helper function to return the current waypoint or [] if none left'''
        if self.current < len(self.palist.poses):
            waypoint = MoveBaseGoal()
            waypoint.target_pose.header.frame_id = "odom"
            waypoint.target_pose.header.stamp = rospy.Time.now()
            waypoint.target_pose.pose = self.palist.poses[self.current]
            return waypoint
        else:
            return []

    def active_cb(self):
        '''This callback is called while a waypoint is being processed'''
        rospy.loginfo("Waypoint "+str(self.current)+" is being processed by the Action Server")

    def feedback_cb(self, feedback):
        '''This callback returns feedback from the planner'''
        rospy.loginfo("Moving towards waypoint "+str(self.current))

    def done_waypoint_cb(self, status, result):
        '''This callback is called when we have reached a waypoint (or failed to reach it)
        We check the status and if successful to far, then move on to next waypoint'''
        self.current += 1
        # Terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html

        if status == 3:
            #If we successfully reached a waypoint, and there are more waypoints, then go to next one:
            rospy.loginfo("Reached waypoint "+str(self.current-1)) 
            next_waypoint = self.get_current_waypoint()
            if next_waypoint:
                rospy.loginfo("Sending waypoint "+str(self.current)+" to Action Server")
                rospy.loginfo(str(next_waypoint.target_pose))
                self.client.send_goal(next_waypoint, self.done_waypoint_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final waypoint reached!")
                self.pub.publish(True)
        elif status == 2:
            rospy.loginfo("Waypoint "+str(self.current)+" received a cancel -- done!")
        elif status == 4:
            rospy.loginfo("Waypoint "+str(self.current)+" was aborted by the Action Server")
        elif status == 5:
            rospy.loginfo("Waypoint "+str(self.current)+" has been rejected by the Action Server")
        elif status == 8:
            rospy.loginfo("Waypoint "+str(self.current)+" received cancel -- cancelled!")
        else:
            rospy.loginfo("Unknown status: "+str(status)+" so quitting")

if __name__ == '__main__':
    rospy.init_node('waypoints_follow')
    try:
        waypoints_follow()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interruption")
    rospy.spin()

