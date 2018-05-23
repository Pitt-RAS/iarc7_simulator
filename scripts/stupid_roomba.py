#!/usr/bin/env python

import rospy

from iarc7_msgs.msg import OdometryArray
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('stupid_roomba')

    pub = rospy.Publisher('/roombas', OdometryArray, queue_size=10)

    vx = 0.33

    msg = OdometryArray()
    roomba = Odometry()
    roomba.header.frame_id = 'map'
    roomba.child_frame_id = 'roomba0/base_link'
    roomba.pose.pose.orientation.w = 1.0
    roomba.twist.twist.linear.x = vx
    msg.data.append(roomba)

    start_time = rospy.Time.now()

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        time = rospy.Time.now()
        time_since_start = (time - start_time).to_sec()

        roomba.pose.pose.position.x = time_since_start * vx
        roomba.header.stamp = time
        pub.publish(msg)

        rate.sleep()
