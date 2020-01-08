#! /usr/bin/env python 
# -*- coding: UTF-8 -*-

import roslib
import sys
import rospy
import numpy as np
import math
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import time
import random
import geometry_msgs
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


if __name__=='__main__':
    rospy.init_node('stop')
    while not rospy.is_shutdown():
		try:
		    robot = ['/r0', '/r1', '/r2', '/r3', '/r4', '/r5']
		    for i in range(1, 6):
		        pub = rospy.Publisher(robot[i] + '/cmd_vel', Twist, queue_size = 10)
		        msg = Twist()
		        msg.linear.x = 0
		        msg.angular.z = 0
		        pub.publish(msg)
		except KeyboardInterrupt:
		    print("shut down")



