#! /usr/bin/env morseexec

"""
Basic MORSE simulation scene for <sim> environment

Feel free to edit this template as you like!
"""

import math
from morse.builder import Environment
from sim.builder.robots import Roomba, Obstacle, Quadcopter

roomba_placement_radius = 1
for i in range(10):
    angle = math.pi / 5 * i
    robot = Roomba('Roomba%i'%i)
    robot.translate(roomba_placement_radius * math.cos(angle),
                    roomba_placement_radius * math.sin(angle),
                    0)
    robot.rotate(0, 0, angle)
    robot.add_default_interface('socket')

obstacle_placement_radius = 5
for i in range(4):
    angle = math.pi / 2 * i
    robot = Obstacle('Obstacle%i'%i)
    robot.translate(obstacle_placement_radius * -math.sin(angle),
                    obstacle_placement_radius * math.cos(angle),
                    0)
    robot.rotate(0, 0, angle)
    robot.add_default_interface('socket')

robot = Quadcopter('Quadcopter')
robot.translate(0, 0, 0.2)
robot.add_default_interface('socket')
robot.add_service('socket')

# set 'fastmode' to True to switch to wireframe mode
env = Environment('environment.blend', fastmode = False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
