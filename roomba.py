import bge
import math
import random
import time

FORWARD_VEL = 0.33
REVERSE_TIME = 20
REVERSE_DURATION = 2.456
NOISE_TIME = 5
NOISE_DURATION = 0.85
NOISE_AMPLITUDE = 20
TURN_45_TIME = REVERSE_TIME / 4.0

ANGULAR_VELOCITY = 3.14 / REVERSE_DURATION

roombas = {}

class Roomba:
    def __init__(self, owner):
        roombas[owner] = self
        self._owner = owner

        self._angular_noise_velocity = 0

        self._bump_signal = False
        self._start_signal = False
        self._wait_signal = False
        self._top_touched = False

        self._last_reverse_time = time.time()
        self._last_noise_time = time.time()
        self._touch_start_time = time.time()

        self._state = 'start'

    def loop(self):
        if self._state == 'start':
            self._last_noise_end_time = time.time()
            self._last_reverse_end_time = time.time()
            self._state = 'run'
        elif self._state == 'wait':
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
                self._owner.setLinearVelocity([FORWARD_VEL, 0, 0], True)
                self._owner.setAngularVelocity([0, 0, 0])
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
                self._owner.setAngularVelocity([0, 0, -ANGULAR_VELOCITY], True)
                self._owner.setLinearVelocity([0, 0, 0])
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
                self._owner.setAngularVelocity([0, 0, -ANGULAR_VELOCITY], True)
                self._owner.setLinearVelocity([0, 0, 0])
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
                self._owner.setAngularVelocity([0, 0, self._angular_noise_velocity], True)
                self._owner.setLinearVelocity([FORWARD_VEL, 0, 0], True)
        else:
            assert False

    def set_radar(self):
        self._bump_signal = True

    def set_touch(self):
        self._top_touched = True

def update():
    obj = bge.logic.getCurrentController().owner
    if not obj in roombas:
        Roomba(obj)
    roombas[obj].loop()

def set_radar():
    controller = bge.logic.getCurrentController()
    if controller.sensors['Radar'].positive:
        roombas[controller.owner].set_radar()

def set_touch():
    controller = bge.logic.getCurrentController()
    if controller.sensors['Touch'].positive:
        roombas[controller.owner].set_touch()
