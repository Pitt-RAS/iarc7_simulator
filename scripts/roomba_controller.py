#! /usr/bin/env python2

import re
from roomba import Roomba, Obstacle
import rospy

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

    for roomba in roombas:
        roomba.send_start_signal()
    for obstacle in obstacles:
        obstacle.send_start_signal()

    rate = rospy.Rate(100)
    while True:
        rate.sleep()
        for roomba in roombas:
            roomba.update()
