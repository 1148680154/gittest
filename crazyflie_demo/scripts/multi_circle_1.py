#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped

class Multi_circle_1():
    def __init__(self, goals):
        rospy.init_node('multi_circle_1', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame1 = rospy.get_param("~frame1")
        #self.frame2 = rospy.get_param("~frame2")
	self.radius = rospy.get_param("~radius")
        self.x = rospy.get_param("~x")
        self.y = rospy.get_param("~y")
        self.z = rospy.get_param("~z")
	self.freq = rospy.get_param("~freq")
	self.lap = rospy.get_param("~lap")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.listener = TransformListener()
        self.goals = goals
        self.goalIndex = 0

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame1, rospy.Time(), rospy.Duration(5.0))
	#rospy.loginfo("start running!")
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        while not rospy.is_shutdown():
            t1 = self.listener.getLatestCommonTime(self.worldFrame, self.frame1)
            if self.listener.canTransform(self.worldFrame, self.frame1, t1):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame1, t1)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
		goal.header.seq += 1
     	        goal.header.stamp = rospy.Time.now()
		#self.x = position[0]
		#self.y = position[1]
		#self.z = position[2]+1
	        goal.pose.position.x = self.x
	        goal.pose.position.y = self.y
	        goal.pose.position.z = self.z
	        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
	        goal.pose.orientation.x = quaternion[0]
	        goal.pose.orientation.y = quaternion[1]
	        goal.pose.orientation.z = quaternion[2]
	        goal.pose.orientation.w = quaternion[3]
		self.pubGoal.publish(goal)
		break

	while not rospy.is_shutdown():
	    #rospy.loginfo("start running!")
            goal.header.seq += 1
            goal.header.stamp = rospy.Time.now()
            self.pubGoal.publish(goal)
            t1 = self.listener.getLatestCommonTime(self.worldFrame, self.frame1)
            if self.listener.canTransform(self.worldFrame, self.frame1, t1):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame1, t1)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
                if     math.fabs(position[0] - self.x) < 0.15 \
                   and math.fabs(position[1] - self.y) < 0.15 \
                   and math.fabs(position[2] - self.z) < 0.15 \
                   and math.fabs(rpy[2] - 0) < math.radians(10) :
			rospy.sleep(3)
		        break

	t_start= rospy.Time.now().to_sec()
	#rospy.loginfo("t_start:%lf",t_start)
        t_now=t_start
        while not rospy.is_shutdown():
 	    goal.header.seq += 1
	    goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = self.x+self.radius*math.sin((t_now-t_start)*2*math.pi*self.freq)
            goal.pose.position.y = self.y+self.radius-self.radius*math.cos((t_now-t_start)*2*math.pi*self.freq)
            goal.pose.position.z = self.z
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]
            self.pubGoal.publish(goal)
	    t_now= rospy.Time.now().to_sec()
	    rospy.loginfo("t_now-t_start:%lf",t_now-t_start)




