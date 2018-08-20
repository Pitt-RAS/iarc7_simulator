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

def start_roombas_callback(data):
    global start_roombas
    start_roombas = True

if __name__ == '__main__':
    rospy.init_node('roomba_controller')

    num_roombas = rospy.get_param('/sim/num_roombas')
    roomba_names = ['/sim/roomba{}/'.format(i) for i in range(num_roombas)]

    num_obstacles = rospy.get_param('/sim/num_obstacles')
    obstacle_names = ['/sim/obstacle{}/'.format(i) for i in range(num_obstacles)]

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

    while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time(0):
        rospy.Rate(10).sleep()

    obstacles = map(Obstacle, obstacle_names)
    roombas = map(Roomba, roomba_names)

    for roomba in roombas:
        roomba.send_start_signal()
    for obstacle in obstacles:
        obstacle.send_start_signal()

    rate = rospy.Rate(100)

    start_roombas = False
    wait_for_start = rospy.get_param('wait_for_start', False)
    if wait_for_start:
        subscriber = rospy.Subscriber('/start_roombas', BoolStamped, start_roombas_callback)
        while not start_roombas:
            rate.sleep()

    while not rospy.is_shutdown():
        rate.sleep()
        for roomba in roombas:
            roomba.update()
