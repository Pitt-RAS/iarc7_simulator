#!/usr/bin/env python
import rospy
import sys
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

rospy.init_node('pose_pub', anonymous=True)
pose = Pose()

pose.position.x = float(sys.argv[1])
pose.position.y = float(sys.argv[2])
pose.position.z = float(sys.argv[3])

orientation = quaternion_from_euler(*map(float, sys.argv[4:7]), axes='rzyx')
pose.orientation.w = orientation[0]
pose.orientation.x = orientation[1]
pose.orientation.y = orientation[2]
pose.orientation.z = orientation[3]

rospy.loginfo(pose)
pub = rospy.Publisher('sim/quad/teleport', Pose, latch=True, queue_size=1)
pub.publish(pose)
rospy.spin()
