#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name","goal")
    r = rospy.get_param("~rate","1")
    x = rospy.get_param("~x","0.0")
    y = rospy.get_param("~y","0.0")
    z = rospy.get_param("~z","1.0")
    yaw = rospy.get_param("~yaw","1.57")
    radius = 0.3
    freq = 0.05
    pi = 3.14159
    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 10
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)
    #s=rospy.Service('start', Empty, start_circle)
    #rospy.Service('stop', Empty, stop_circle)

    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
	#print(rospy.Time.now().to_sec())
        #msg.pose.position.x = radius*math.sin(2*math.pi*freq*msg.header.stamp.to_sec())+x
	#msg.pose.position.y = radius-radius*math.cos(2*math.pi*freq*msg.header.stamp.to_sec())+y
        pub.publish(msg)
        rate.sleep()
