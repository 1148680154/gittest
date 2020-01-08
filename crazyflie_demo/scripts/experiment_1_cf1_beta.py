#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('experimnet_1_cf1_beta', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    frame1 = rospy.get_param("~frame1")
    frame2 = rospy.get_param("~frame2")
    v1 = rospy.get_param("~vel")
    gama = rospy.get_param("~gama")
    N = rospy.get_param("~N")
    w0 = rospy.get_param("~w0")
    pub = rospy.Publisher('goal', PoseStamped, queue_size=1)
    listener = TransformListener()
    x1 = rospy.get_param("~x")
    y1 = rospy.get_param("~y")
    z1 = rospy.get_param("~z")

    pre_pose_1 = PoseStamped()
    pre_pose_2 = PoseStamped()
    pre_time_1 = rospy.Time.now()
    pre_time_2 = rospy.Time.now()

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x1
    msg.pose.position.y = y1
    msg.pose.position.z = z1
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    listener.waitForTransform(worldFrame, frame1, rospy.Time(), rospy.Duration(5.0))
    listener.waitForTransform(worldFrame, frame2, rospy.Time(), rospy.Duration(5.0))
    
    # wait for cf1 to reach initial pose
    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
	t = listener.getLatestCommonTime(worldFrame, frame1)
        if listener.canTransform(worldFrame, frame1, t):
            position, quaternion = listener.lookupTransform(worldFrame, frame1, t)
            rpy = tf.transformations.euler_from_quaternion(quaternion)
            if     math.fabs(position[0] - x1) < 0.15 \
               and math.fabs(position[1] - y1) < 0.15 \
               and math.fabs(position[2] - z1) < 0.15 \
               and math.fabs(rpy[2] - 0) < math.radians(10) :
                   rospy.sleep(5)
	   	   break

    # set pre_pose_1 & pre_pose_2
    while not rospy.is_shutdown():
	t1 = listener.getLatestCommonTime(worldFrame, frame1)
	t2 = listener.getLatestCommonTime(worldFrame, frame2)
        if (listener.canTransform(worldFrame, frame1, t1) and listener.canTransform(worldFrame, frame2, t2)):
            position1, quaternion1 = listener.lookupTransform(worldFrame, frame1, t1)
	    position2, quaternion2 = listener.lookupTransform(worldFrame, frame2, t2)
	    pre_pose_1.pose.position.x = position1[0]
	    pre_pose_1.pose.position.y = position1[1]
	    pre_pose_1.pose.position.z = position1[2]
	    pre_pose_2.pose.position.x = position2[0]
	    pre_pose_2.pose.position.y = position2[1]
	    pre_pose_2.pose.position.z = position2[2]
	    pre_time_1 = t1
	    pre_time_2 = t2
	    break

    # calculation of velocity and omega
    #t_start= rospy.Time.now().to_sec()
    #t_now=t_start
    rospy.loginfo("start to follow") 
    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
	#t=t_now-t_start
        t1 = listener.getLatestCommonTime(worldFrame, frame1)
	t2 = listener.getLatestCommonTime(worldFrame, frame2)
	if (listener.canTransform(worldFrame, frame1, t1) and listener.canTransform(worldFrame, frame2, t2)):
	    position1, quaternion1 = listener.lookupTransform(worldFrame, frame1, t1)	 	
	    position2, quaternion2 = listener.lookupTransform(worldFrame, frame2, t2)	
	    #t1 = rospy.Time.now()
 	    #t2 = rospy.Time.now() 	
	    dt1 = t1-pre_time_1
	    dt2 = t2-pre_time_2
	    #position1[0] = x1
    	    #position1[1] = y1    
	    dx1 = x1 - pre_pose_1.pose.position.x
	    dy1 = y1 - pre_pose_1.pose.position.y
	    dx2 = position2[0] - pre_pose_2.pose.position.x
	    dy2 = position2[1] - pre_pose_2.pose.position.y
	    theta1 = math.atan2(dy1,dx1)
	    theta2 = math.atan2(dy2,dx2)
	    pre_time_1 = t1
	    pre_time_2 = t2
	    u1 = w0 + gama / N * math.sin(theta2 - theta1)
    	    #u2 = w0 + gama / N * math.sin(theta1 - theta2) 
	    theta1 = theta1 + u1 * (dt1.to_sec())
    	    #theta2 = theta2 + u2 * (dt2.to_sec())
	    pre_pose_1.pose.position.x = position1[0]
	    pre_pose_1.pose.position.y = position1[1]
 	    pre_pose_2.pose.position.x = position2[0]
	    pre_pose_2.pose.position.y = position2[1]
	    x1 = x1 + v1 * (dt1.to_sec()) * math.cos(theta1)
            y1 = y1 + v1 * (dt2.to_sec()) * math.sin(theta1)
            #x2 = x2 + v2 * dt2 * math.cos(theta2)
            #y2 = y2 + v2 * dt2 * math.sin(theta2)
            x1 = max(min(x1, 2), -2)
            y1 = max(min(y1, 2), -2)
	    msg.pose.position.x = x1
            msg.pose.position.y = y1
            msg.pose.position.z = z1
	    #t_now= rospy.Time.now().to_sec()
	    pub.publish(msg)
	    rospy.loginfo("x:%lf  y:%lf",msg.pose.position.x,msg.pose.position.y)
	    rospy.sleep(0.1)


"""
class Circle():
    def __init__(self, goals):
        rospy.init_node('circle', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
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
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
	#rospy.loginfo("start running!")
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame

        t_start= rospy.Time.now().to_sec()
	#rospy.loginfo("t_start:%lf",t_start)
        t_now=t_start
        while (not rospy.is_shutdown()):
 	    goal.header.seq += 1
	    goal.header.stamp = rospy.Time.now()
            t=t_now-t_start
            u1=w+r*sin(a2-a1)
            u2=w+r*sin(a1-a2)           
            a1=(a1+u1*t)%xx
            a2=(a2+u2*t)%xx
            x1+=v1*t*cosa1
            y1+=v1*t*sina1
            x2+=v2*t*cosa2
            y2+=v2*t*sina2
            goal.pose.position.x = x1
            goal.pose.position.y = y1
            goal.pose.position.z = self.z
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex-1][3])
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]
            self.pubGoal.publish(goal)
	    t_now= rospy.Time.now().to_sec()
	    rospy.loginfo("t_now-t_start:%lf",t_now-t_start)

x1,y1,x2,y2,v1,v2,a1,a2
"""

