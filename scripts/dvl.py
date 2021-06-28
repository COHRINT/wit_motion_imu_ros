#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np

rospy.init_node("dvl_sim")
pub = rospy.Publisher("dvl", TwistWithCovarianceStamped, queue_size=1)
r = rospy.Rate(10)
seq = 0

while not rospy.is_shutdown():
    msg = TwistWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "base_link"
    msg.header.seq = seq

    msg.twist.twist.linear.x = 0 + np.random.normal(scale=0.01)
    msg.twist.twist.linear.y = 0 + np.random.normal(scale=0.01)
    msg.twist.twist.linear.z = 0 + np.random.normal(scale=0.01)
    cov = np.diag([0.1, 0.1, 0.1, -1, -1, -1])
    msg.twist.covariance = list(cov.flatten())

    pub.publish(msg)

    seq += 1
    r.sleep()