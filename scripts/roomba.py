import itertools
import math
import random
import time

# All units are mks

FORWARD_VEL = 0.33
REVERSE_TIME = 20
REVERSE_DURATION = 2.456
NOISE_TIME = 5
NOISE_DURATION = 0.85
NOISE_AMPLITUDE = 20
TURN_45_TIME = REVERSE_TIME / 4.0

ANGULAR_VELOCITY = 3.14 / REVERSE_DURATION

OBSTACLE_CIRCLE_RADIUS = 5

class Obstacle:
    def __init__(self, owner):
        self._owner = owner

        self._start_signal = False
        self._wait_signal = False

        # Each element is the flag for a specific bumper
        self._bumper_data = []
        self._bumper_on = False

        self._state = 'wait'

        i = 0
        for component_name in self._owner:
            if 'Bumper' in component_name:
                callback = lambda data, i = i: self._update_bumper(data, i)
                getattr(self._owner, component_name).subscribe(callback)
                self._bumper_data.append(False)
                i += 1

    def loop(self):
        if self._state == 'wait':
            if self._start_signal:
                self._state = 'run'
                self._start_signal = False
        elif self._state == 'run':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif self._bumper_on:
                self._owner.motion.publish({'v': 0, 'w': 0})
            else:
                self._owner.motion.publish({'v': FORWARD_VEL, 'w': -FORWARD_VEL / OBSTACLE_CIRCLE_RADIUS})

    def send_start_signal(self):
        self._start_signal = True

    def send_wait_signal(self):
        self._wait_signal = True

    def _update_bumper(self, sensor_data, bumper_index):
        self._bumper_data[bumper_index] = sensor_data['positive']
        self._bumper_on = (True in self._bumper_data)

class Roomba:
    def __init__(self, owner):
        self._owner = owner

        self._angular_noise_velocity = 0

        self._bump_signal = False
        self._start_signal = False
        self._wait_signal = False
        self._top_touched = False

        self._last_reverse_time = time.time()
        self._last_noise_time = time.time()
        self._touch_start_time = time.time()

        self._state = 'wait'

        for component_name in self._owner:
            if 'Bumper' in component_name:
                getattr(self._owner, component_name).subscribe(lambda data: self._update_bumper(data))

    def loop(self):
        if self._state == 'wait':
            if self._start_signal:
                self._state = 'run'
                self._start_signal = False
                self._last_noise_end_time = time.time()
                self._last_reverse_end_time = time.time()
        elif self._state == 'run':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif self._top_touched:
                self._state = 'touched'
                self._top_touched = False
                self._touch_start_time = time.time()
            elif time.time() - self._last_reverse_time >= REVERSE_TIME:
                self._state = 'reverse'
                self._last_reverse_time = time.time()
            elif time.time() - self._last_noise_time >= NOISE_TIME:
                self._state = 'noise'
                self._angular_noise_velocity = random.uniform(-NOISE_AMPLITUDE, NOISE_AMPLITUDE) / NOISE_DURATION * (math.pi / 180)
                self._last_noise_time = time.time()
            elif self._bump_signal:
                self._state = 'reverse'
                self._bump_signal = False
                self._last_reverse_time = time.time()
            else:
                self._owner.motion.publish({'v': FORWARD_VEL, 'w': 0})
        elif self._state == 'touched':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif time.time() - self._touch_start_time >= TURN_45_TIME:
                self._state = 'run'
            elif self._bump_signal:
                self._state = 'reverse'
                self._bump_signal = False
                self._last_reverse_time = time.time()
            else:
                self._owner.motion.publish({'v': 0, 'w': -ANGULAR_VELOCITY})
        elif self._state == 'reverse':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif self._top_touched:
                self._state = 'touched'
                self._top_touched = False
                self._touch_start_time = time.time()
            elif time.time() - self._last_reverse_time >= REVERSE_DURATION:
                self._state = 'run'
            else:
                self._owner.motion.publish({'v': 0, 'w': -ANGULAR_VELOCITY})
            self._bump_signal = False
        elif self._state == 'noise':
            if self._wait_signal:
                self._state = 'wait'
                self._wait_signal = False
            elif self._top_touched:
                self._state = 'touched'
                self._top_touched = False
                self._touch_start_time = time.time()
            elif time.time() - self._last_noise_time >= NOISE_DURATION:
                self._state = 'run'
            elif self._bump_signal:
                self._state = 'reverse'
                self._bump_signal = False
                self._last_reverse_time = time.time()
            else:
                self._owner.motion.publish({'v': FORWARD_VEL, 'w': self._angular_noise_velocity})
        else:
            assert False

    def send_start_signal(self):
        self._start_signal = True

    def send_wait_signal(self):
        self._wait_signal = True

    def _update_bumper(self, sensor_data):
        if sensor_data['positive']:
            self._bump_signal = True

    def _update_touch(self, sensor_data):
        if sensor_data['positive']:
            self._top_touched = True
