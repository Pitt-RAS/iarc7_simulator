#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

from iarc7_msgs.msg import OdometryArray

def callback(msg):
    marker = Marker()

    if not msg.data:
        marker.header.frame_id = 'map'
    else:
        marker.header = msg.data[0].header

    marker.ns = 'roomba_observations'
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.MODIFY

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 0.8
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration(0.5)

    marker.frame_locked = False

    for odom in msg.data:
        marker.points.append(odom.pose.pose.position)

    pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('publish_observed_roomba_markers')
    rospy.Subscriber('/roomba_observations', OdometryArray, callback)
    pub = rospy.Publisher('/roomba_observation_markers', Marker, queue_size=5)
    rospy.spin()
