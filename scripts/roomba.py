from __future__ import division

import itertools
import math
import random
import re
import rospy
import time

from iarc7_msgs.msg import BoolStamped
from geometry_msgs.msg import Twist

# All units are mks

FORWARD_VEL = 0.33
REVERSE_TIME = rospy.Duration(20.0)
REVERSE_DURATION = rospy.Duration(2.456)
NOISE_TIME = rospy.Duration(5.0)
NOISE_DURATION = rospy.Duration(0.85)
NOISE_AMPLITUDE = 20
TURN_45_TIME = REVERSE_TIME / 4.0

ANGULAR_VELOCITY = math.pi / REVERSE_DURATION.to_sec()

OBSTACLE_CIRCLE_RADIUS = 5

class Obstacle:
    def __init__(self, namespace):
        self._start_signal = False
        self._wait_signal = False

        # Each element is the flag for a specific bumper
        self._bumper_data = []
        self._bumper_on = False

        self._state = 'wait'

        self._publisher = rospy.Publisher('{}cmd_vel'.format(namespace),
                                          Twist,
                                          queue_size=10)

        for topic, _ in rospy.get_published_topics(namespace):
            match = re.match('{}bumper([0-9]+)'.format(namespace), topic)
            if match:
                def bumper_callback(msg, n=int(match.group(1))):
                    self._update_bumper(msg.data, n)

                rospy.Subscriber(match.group(),
                                 BoolStamped,
                                 bumper_callback)

                self._bumper_data.append(False)

    def _publish_speed(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._publisher.publish(twist)

    def _update(self):
        if self._state == 'wait':
            if self._start_signal:
                self._state = 'run'
                self._start_signal = False
        elif self._state == 'run':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif self._bumper_on:
                self._publish_speed(0, 0)
            else:
                self._publish_speed(FORWARD_VEL,
                                    -FORWARD_VEL / OBSTACLE_CIRCLE_RADIUS)

    def send_start_signal(self):
        self._start_signal = True
        self._update()

    def send_wait_signal(self):
        self._wait_signal = True
        self._update()

    def _update_bumper(self, sensor_data, bumper_index):
        self._bumper_data[bumper_index] = sensor_data
        self._bumper_on = (True in self._bumper_data)
        self._update()

class Roomba:
    def __init__(self, namespace):
        self._angular_noise_velocity = 0

        self._bump_signal = False
        self._start_signal = False
        self._wait_signal = False
        self._top_touched = False

        self._last_reverse_time = rospy.Time.now()
        self._last_noise_time = rospy.Time.now()
        self._touch_start_time = rospy.Time.now()

        self._state = 'wait'

        self._publisher = rospy.Publisher('{}cmd_vel'.format(namespace),
                                          Twist,
                                          queue_size=10)

        self.thingy = rospy.Subscriber('{}top_touch'.format(namespace),
                         BoolStamped,
                         lambda msg: self._update_touch(msg.data))

        self.things = []
        for topic, _ in rospy.get_published_topics(namespace):
            match = re.match('{}bumper([0-9]+)'.format(namespace), topic)
            if match:
                def bumper_callback(msg):
                    self._update_bumper(msg.data)

                self.things.append(rospy.Subscriber(match.group(),
                                 BoolStamped,
                                 bumper_callback))

    def _publish_speed(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._publisher.publish(twist)

    def update(self):
        if self._state == 'wait':
            if self._start_signal:
                self._state = 'run'
                self._start_signal = False
                self._last_noise_end_time = rospy.Time.now()
                self._last_reverse_end_time = rospy.Time.now()
        elif self._state == 'run':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif self._top_touched:
                self._state = 'touched'
                self._top_touched = False
                self._touch_start_time = rospy.Time.now()
            elif rospy.Time.now() - self._last_reverse_time >= REVERSE_TIME:
                self._state = 'reverse'
                self._last_reverse_time = rospy.Time.now()
            elif rospy.Time.now() - self._last_noise_time >= NOISE_TIME:
                self._state = 'noise'
                self._angular_noise_velocity = (random.uniform(-NOISE_AMPLITUDE,
                                                              NOISE_AMPLITUDE)
                                              / NOISE_DURATION.to_sec()
                                              * (math.pi / 180))
                self._last_noise_time = rospy.Time.now()
            elif self._bump_signal:
                self._state = 'reverse'
                self._bump_signal = False
                self._last_reverse_time = rospy.Time.now()
            else:
                self._publish_speed(FORWARD_VEL, 0)
        elif self._state == 'touched':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif rospy.Time.now() - self._touch_start_time >= TURN_45_TIME:
                self._state = 'run'
            elif self._bump_signal:
                self._state = 'reverse'
                self._bump_signal = False
                self._last_reverse_time = rospy.Time.now()
            else:
                self._publish_speed(0, -ANGULAR_VELOCITY)
        elif self._state == 'reverse':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif self._top_touched:
                self._state = 'touched'
                self._top_touched = False
                self._touch_start_time = rospy.Time.now()
            elif rospy.Time.now() - self._last_reverse_time >= REVERSE_DURATION:
                self._state = 'run'
            else:
                self._publish_speed(0, -ANGULAR_VELOCITY)
            self._bump_signal = False
        elif self._state == 'noise':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif self._top_touched:
                self._state = 'touched'
                self._top_touched = False
                self._touch_start_time = rospy.Time.now()
            elif rospy.Time.now() - self._last_noise_time >= NOISE_DURATION:
                self._state = 'run'
            elif self._bump_signal:
                self._state = 'reverse'
                self._bump_signal = False
                self._last_reverse_time = rospy.Time.now()
            else:
                self._publish_speed(FORWARD_VEL, self._angular_noise_velocity)
        else:
            assert False

    def send_start_signal(self):
        self._start_signal = True

    def send_wait_signal(self):
        self._wait_signal = True

    def _update_bumper(self, sensor_data):
        if sensor_data:
            self._bump_signal = True

    def _update_touch(self, sensor_data):
        if sensor_data:
            self._top_touched = True
