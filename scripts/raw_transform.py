#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
# from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import tf
from sensor_msgs.msg import Imu




rospy.init_node("ori_sim")
pub = rospy.Publisher("ori", PoseWithCovarianceStamped, queue_size=1)
r = rospy.Rate(0.5)
bias = None

def callback(in_msg):
    global bias
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([in_msg.orientation.x, in_msg.orientation.y, in_msg.orientation.z, in_msg.orientation.w])
    if bias is None:
        print("bias: {}".format(bias))
        bias = roll
    new_yaw = -(roll - bias)
    while new_yaw > np.pi:
        new_yaw -= 2 * np.pi
    while new_yaw < -np.pi:
        new_yaw += 2 * np.pi

    print(np.degrees(new_yaw))


    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "odom"

    q = tf.transformations.quaternion_from_euler(0, 0, new_yaw)

    msg.pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    cov = np.diag([-1,-1,-1,0.5,0.1,0.1])
    msg.pose.covariance = list(cov.flatten())
    pub.publish(msg)

rospy.Subscriber("imu/data_raw", Imu, callback)

while not rospy.is_shutdown():
    r.sleep()
