#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

rospy.init_node("wit_imu_stub")
pub = rospy.Publisher("imu/data", Imu, queue_size=10)

def callback(msg):
    global pub

    imu = Imu()
    imu.header.frame_id = "base_link"
    imu.header.stamp = msg.header.stamp
    imu.orientation = msg.pose.pose.orientation
    pub.publish( imu )


rospy.Subscriber("pose_gt", Odometry, callback)
rospy.spin()