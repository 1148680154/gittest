#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped

class Circle():
    def __init__(self, goals):
        rospy.init_node('circle', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
	self.radius = rospy.get_param("~radius")
	self.freq = rospy.get_param("~freq")
        self.x = rospy.get_param("~x")
        self.y = rospy.get_param("~y")
        self.z = rospy.get_param("~z")
	self.lap = rospy.get_param("~lap")
	self.omega = self.freq*2*3.1415926
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.listener = TransformListener()
        self.goals = goals
        self.goalIndex = 0

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
	#rospy.loginfo("start running!")
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        while not rospy.is_shutdown():
	    #rospy.loginfo("start running!")
            goal.header.seq += 1
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = self.x
            goal.pose.position.y = self.y
            goal.pose.position.z = self.z
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]

            self.pubGoal.publish(goal)

            t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
            if self.listener.canTransform(self.worldFrame, self.frame, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
                if     math.fabs(position[0] - self.x) < 0.15 \
                   and math.fabs(position[1] - self.y) < 0.15 \
                   and math.fabs(position[2] - self.z) < 0.15 \
                   and math.fabs(rpy[2] - self.goals[self.goalIndex][3]) < math.radians(10) \
                   and self.goalIndex < len(self.goals) - 2:
                        rospy.sleep(2)
                        self.goalIndex += 1
			#rospy.loginfo("Index:%lf",self.goalIndex)
			if self.goalIndex == len(self.goals) - 2:
			    break

	t_start= rospy.Time.now().to_sec()
	#rospy.loginfo("t_start:%lf",t_start)
        t_now=t_start
        while ((not rospy.is_shutdown()) and (t_now-t_start<self.lap/self.freq)):
 	    goal.header.seq += 1
	    goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = self.x+self.radius*math.sin((t_now-t_start)*self.omega)
            goal.pose.position.y = self.y+self.radius-self.radius*math.cos((t_now-t_start)*self.omega)
            goal.pose.position.z = self.z
            quaternion = tf.transformations.quaternion_from_euler(0, 0, (t_now-t_start)*self.omega)
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]
            self.pubGoal.publish(goal)
	    t_now= rospy.Time.now().to_sec()
	    #rospy.loginfo("t_now-t_start:%lf",t_now-t_start)

      	while not rospy.is_shutdown():
            goal.header.seq += 1
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = self.x
            goal.pose.position.y = self.y
            goal.pose.position.z = self.z
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]

            self.pubGoal.publish(goal)
	    #rospy.loginfo("Index:%lf",self.goalIndex)

            t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
            if self.listener.canTransform(self.worldFrame, self.frame, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
                if     math.fabs(position[0] - self.x) < 0.15 \
                   and math.fabs(position[1] - self.y) < 0.15 \
                   and math.fabs(position[2] - self.z) < 0.15 \
                   and math.fabs(rpy[2] - self.goals[self.goalIndex][3]) < math.radians(10) \
                   and self.goalIndex < len(self.goals) - 1:
                        rospy.sleep(self.goals[self.goalIndex][4])
                        self.goalIndex += 1

