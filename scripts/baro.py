#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

rospy.init_node("baro_sim")
pub = rospy.Publisher("baro", PoseWithCovarianceStamped, queue_size=1)
r = rospy.Rate(50)
seq = 0

while not rospy.is_shutdown():
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "odom"
    msg.header.seq = seq

    msg.pose.pose.position.z = -0.1 + np.random.normal(scale=0.01)
    cov = np.diag([-1,-1,0.1,-1,-1,-1])
    msg.pose.covariance = list(cov.flatten())

    pub.publish(msg)

    seq += 1
    r.sleep()