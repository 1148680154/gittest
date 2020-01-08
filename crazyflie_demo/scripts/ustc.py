#!/usr/bin/env python

import numpy as np
import csv
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

start=0 

def start(req):
    global start
    start = 1
    print "start to publish pose!"
    return []

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name","goal")
    r = rospy.get_param("~rate","1.0")
    x0 = rospy.get_param("~x0","0.0")
    y0 = rospy.get_param("~y0","0.0")
    z0 = rospy.get_param("~z0","1.0")
    csv_file = rospy.get_param("~csv_file")
    
    rate = rospy.Rate(r)
    routes = np.loadtxt(csv_file,skiprows=1,delimiter=',') #routes of all cfs
    ids = np.loadtxt(csv_file,str,skiprows=1,delimiter=',',usecols=[0]) #ids of all cfs
    pose_index = 1 
    pose_num = (routes.shape[1]-1)/4
    pub_count = 0
    msgs = range(0,routes.shape[0])  #its elements are set to PoseStamped later
    pubs = range(0,routes.shape[0])  #its elements are set to Publisher later
    s = rospy.Service('start', Empty, start)
    #rospy.spin()
    for i in range(1,routes.shape[0]):
        msgs[i] = PoseStamped()
        msgs[i].header.seq = 0
        msgs[i].header.stamp = rospy.Time.now()
        msgs[i].header.frame_id = worldFrame
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        msgs[i].pose.orientation.x = quaternion[0]
        msgs[i].pose.orientation.y = quaternion[1]
        msgs[i].pose.orientation.z = quaternion[2]
        msgs[i].pose.orientation.w = quaternion[3]
        pubs[i] = rospy.Publisher("crazyflie"+ids[i][0]+"/"+"goal", PoseStamped, queue_size=1)
   
    while start !=1 and (not rospy.is_shutdown()):
        start=start
    print routes
    
    while (not rospy.is_shutdown()) and (pose_index<=pose_num):
        for i in range(1,routes.shape[0]):
            msgs[i].header.seq += 1
            msgs[i].header.stamp = rospy.Time.now()
            msgs[i].pose.position.x = routes[i][4*pose_index-3] + x0
            msgs[i].pose.position.y = routes[i][4*pose_index-2] + y0
            msgs[i].pose.position.z = routes[i][4*pose_index-1] + z0
            pubs[i].publish(msgs[i])
        pub_count += 1
        if (pub_count>routes[1][4*pose_index]*r):
            pose_index = pose_index % pose_num + 1
            pub_count = 0
        rate.sleep()


