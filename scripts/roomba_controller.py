#! /usr/bin/env python2

import re
import rospy

from iarc7_msgs.msg import BoolStamped
from iarc7_msgs.srv import SetBoolOn, SetBoolOnResponse

from roomba import Roomba, Obstacle

def top_touch_service_handler(request):
    try:
        message = BoolStamped()
        message.header.stamp = rospy.Time.now()
        message.data = request.data
        roomba_bump_publishers[request.id].publish(message)
    except KeyError:
        error_msg = 'Tap requested on non-existant roomba: {}'.format(request.id)
        rospy.logerr(error_msg)
        return SetBoolOnResponse(success=False, message=error_msg)
    return SetBoolOnResponse(success=True)

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

    roomba_bump_publishers = dict()

    for roomba in roomba_names:
        name = '{}top_touch'.format(roomba)
        pub = rospy.Publisher(name,
                              BoolStamped,
                              queue_size=0)
        index = re.sub('^\/sim\/', '', name)
        roomba_bump_publishers[index] = pub

        name = '{}bumper'.format(roomba)
        pub = rospy.Publisher('{}0'.format(name),
                              BoolStamped,
                              queue_size=0)
        index = re.sub('^\/sim\/', '', name)
        roomba_bump_publishers[index] = pub

    rospy.Service('/sim/roomba_bumper_tap', SetBoolOn, top_touch_service_handler)

    obstacles = map(Obstacle, obstacle_names)
    roombas = map(Roomba, roomba_names)

    for roomba in roombas:
        roomba.send_start_signal()
    for obstacle in obstacles:
        obstacle.send_start_signal()

    rate = rospy.Rate(100)
    while True:
        rate.sleep()
        for roomba in roombas:
            roomba.update()
