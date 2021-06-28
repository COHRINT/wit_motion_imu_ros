#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
# from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

rospy.init_node("ori_sim")
pub = rospy.Publisher("ori", PoseWithCovarianceStamped, queue_size=1)
r = rospy.Rate(10)
seq = 0

while not rospy.is_shutdown():
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "odom"
    msg.header.seq = seq

    msg.pose.pose.orientation = Quaternion(0,0,0,1)
    cov = np.diag([-1,-1,-1,0.1,0.1,-1])
    msg.pose.covariance = list(cov.flatten())
    pub.publish(msg)

    seq += 1
    r.sleep()