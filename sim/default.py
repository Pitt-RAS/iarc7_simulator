#! /usr/bin/env morseexec

"""
Basic MORSE simulation scene for <sim> environment

Feel free to edit this template as you like!
"""

from morse.builder import Environment, Quadrotor
from sim.builder.robots import Roomba, Obstacle

for i in range(10):
    robot = Roomba('Roomba' + str(i))
    robot.translate(0, i * 0.5, 0)
    robot.add_default_interface('socket')

quad = Quadrotor()
quad.translate(1, 0, 0)

# set 'fastmode' to True to switch to wireframe mode
env = Environment('environment.blend', fastmode = False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
