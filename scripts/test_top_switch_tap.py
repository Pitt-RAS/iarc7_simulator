#!/usr/bin/env python

import rospy

from iarc7_msgs.msg import BoolStamped
from iarc7_msgs.srv import SetBoolOn, SetBoolOnRequest

if __name__ == '__main__':
    rospy.init_node('test_top_switch_tap')

    rospy.wait_for_service('/sim/roomba_top_switch_tap')
    tap_service = rospy.ServiceProxy('/sim/roomba_top_switch_tap', SetBoolOn)

    for i in range(-3,13):
        try:
            response = tap_service(str(i), True)
            rospy.loginfo('Tap on roomba %d success: %s', i, response.success)
        except rospy.ServiceException as exc:
            rospy.logerr('Could not tap roomba: %s', i)
