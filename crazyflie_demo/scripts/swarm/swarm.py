#!/usr/bin/env python
import csv
import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped

class Swarm():
    def __init__(self):
        rospy.init_node('demo', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.listener = TransformListener()
        self.goals = goals
        self.goalIndex = 1
        self.index = 1
        with open('test.csv','rb') as myfile:
	          reader=csv.reader(myfile)
	          lines = [line for line in reader]

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        while not rospy.is_shutdown():
            goal.header.seq += 1
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = int(lines[self.goalIndex][3*index-1])
            goal.pose.position.y = int(lines[self.goalIndex][3*index+0])
            goal.pose.position.z = int(lines[self.goalIndex][3*index+1])
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            goal.pose.orientation.x = 0
            goal.pose.orientation.y = 0
            goal.pose.orientation.z = 0
            goal.pose.orientation.w = 1

            self.pubGoal.publish(goal)

            t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
            if self.listener.canTransform(self.worldFrame, self.frame, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
                if     math.fabs(position[0] - int(lines[self.goalIndex][3*index-1])) < 0.25 \
                   and math.fabs(position[1] - int(lines[self.goalIndex][3*index+0])) < 0.25 \
                   and math.fabs(position[2] - int(lines[self.goalIndex][3*index+1])) < 0.25 \
                   and self.goalIndex < len(lines):
                        rospy.sleep(lines[self.goalIndex][1])
                        self.goalIndex += 1

if __name__ == '__main__':
    demo = Swarm()
    demo.run()

