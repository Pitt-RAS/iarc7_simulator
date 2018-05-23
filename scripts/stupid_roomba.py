#!/usr/bin/env python

import rospy
import tf2_ros as tf2

from geometry_msgs.msg import TransformStamped
from iarc7_msgs.msg import OdometryArray
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('stupid_roomba')

    tf2_broadcaster = tf2.TransformBroadcaster()

    pub = rospy.Publisher('/roombas', OdometryArray, queue_size=10)

    vx = 0.33

    roomba_msg = OdometryArray()
    roomba = Odometry()
    roomba.header.frame_id = 'map'
    roomba.child_frame_id = 'roomba0/base_link'
    roomba.pose.pose.orientation.w = 1.0
    roomba.twist.twist.linear.x = vx
    roomba_msg.data.append(roomba)

    tf_msg = TransformStamped()
    tf_msg.header.frame_id = 'map'
    tf_msg.child_frame_id = 'roomba0/base_link'
    tf_msg.transform.rotation.w = 1.0

    start_time = rospy.Time.now()

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        time = rospy.Time.now()
        time_since_start = (time - start_time).to_sec()

        roomba.pose.pose.position.x = time_since_start * vx
        roomba.header.stamp = time
        pub.publish(roomba_msg)

        tf_msg.header.stamp = time
        tf_msg.transform.translation.x = time_since_start * vx
        tf2_broadcaster.sendTransform(tf_msg)

        rate.sleep()
