#! /usr/bin/env python2

import re
import rospy

from iarc7_msgs.msg import BoolStamped
from iarc7_msgs.srv import SetBoolOn, SetBoolOnResponse

from roomba import Roomba, Obstacle

def top_touch_service_handler(request):
    # Don't use exception checking here because we also need to check for
    # keys less than 0
    id = int(request.id)
    if id > -1 and id < len(roomba_top_tap_publishers):
        message = BoolStamped()
        message.header.stamp = rospy.Time.now()
        message.data = request.data
        roomba_top_tap_publishers[id].publish(message)
        return SetBoolOnResponse(success=True)
    else:
        error_msg = 'Tap requested on non-existant roomba'
        rospy.logerr(error_msg)
        return SetBoolOnResponse(success=False, message=error_msg)

if __name__ == '__main__':
    rospy.init_node('roomba_controller')

    roomba_names = set()
    obstacle_names = set()

    for topic_name, topic_type in rospy.get_published_topics():
        match = re.match('(/sim/obstacle[0-9]*/).+', topic_name)
        if match:
            obstacle_names.add(match.group(1))

        match = re.match('(/sim/roomba[0-9]*/).+', topic_name)
        if match:
            roomba_names.add(match.group(1))

    obstacles = map(Obstacle, obstacle_names)
    roombas = map(Roomba, roomba_names)

    roomba_names = sorted(roomba_names)

    roomba_top_tap_publishers = []
    for roomba in roomba_names:
        pub = rospy.Publisher('{}top_touch'.format(roomba),
                              BoolStamped,
                              queue_size=0)
        roomba_top_tap_publishers.append(pub)

    rospy.Service('/sim/roomba_top_switch_tap', SetBoolOn, top_touch_service_handler)

    for roomba in roombas:
        roomba.send_start_signal()
    for obstacle in obstacles:
        obstacle.send_start_signal()

    rate = rospy.Rate(100)
    while True:
        rate.sleep()
        for roomba in roombas:
            roomba.update()
