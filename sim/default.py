#! /usr/bin/env morseexec

import math
from morse.builder import Clock, Environment, FakeRobot
from morse.core.morse_time import TimeStrategies
from sim.builder.robots import TargetRoomba, Obstacle, Quadcopter

# Get ROS parameters
import rospy
floor_material_name = rospy.get_param('sim/floor')
front_camera_resolution = rospy.get_param('sim/front_camera_resolution', None)
left_camera_resolution = rospy.get_param('sim/left_camera_resolution', None)
right_camera_resolution = rospy.get_param('sim/right_camera_resolution', None)
back_camera_resolution = rospy.get_param('sim/back_camera_resolution', None)
bottom_camera_resolution = rospy.get_param('sim/bottom_camera_resolution', None)
create_teleport_actuator = rospy.get_param('sim/create_teleport_actuator', False)

num_targets = rospy.get_param('sim/num_roombas')
num_obstacles = rospy.get_param('sim/num_obstacles')

# Place roombas
roomba_placement_radius = 1
for i in range(num_targets):
    angle = 2*math.pi / num_targets * i
    robot = TargetRoomba('roomba%i'%i)
    robot.translate(roomba_placement_radius * math.cos(angle),
                    roomba_placement_radius * math.sin(angle),
                    0)
    robot.rotate(0, 0, angle)

# Place obstacles
obstacle_placement_radius = 5
for i in range(num_obstacles):
    angle = 2*math.pi / num_obstacles * i
    robot = Obstacle('obstacle%i'%i)
    robot.translate(obstacle_placement_radius * -math.sin(angle),
                    obstacle_placement_radius * math.cos(angle),
                    0)
    robot.rotate(0, 0, angle)

# Place drone
robot = Quadcopter('Quadcopter',
                   front_camera_resolution=front_camera_resolution,
                   left_camera_resolution=left_camera_resolution,
                   right_camera_resolution=right_camera_resolution,
                   back_camera_resolution=back_camera_resolution,
                   bottom_camera_resolution=bottom_camera_resolution,
                   create_teleport_actuator=create_teleport_actuator)
robot.translate(0, 0, 0.2)

# Add /clock publisher
fake_robot = FakeRobot()
clock = Clock()
clock.add_interface('ros', topic='/clock')
fake_robot.append(clock)

# set 'fastmode' to True to switch to wireframe mode
env = Environment('environment.blend', fastmode = False)

# These have to be set for the imu (really the magnetometer in the imu) to work.
env.properties(latitude=0.0, longitude=0.0, altitude=0.0)

env.set_time_strategy(TimeStrategies.FixedSimulationStep)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])

# Set floor material
# Must be run after environment is loaded
import bpy
mat = bpy.data.materials.get(floor_material_name)
bpy.data.objects['Plane'].data.materials[0] = mat

# Alternate roomba colors
green_mat = bpy.data.materials.get("Gloss Banner Green")
red_mat = bpy.data.materials.get("Gloss Banner Red")
mats = [green_mat, red_mat]
for i in range(num_targets):
    bpy.data.objects["roomba%d"%i].data.materials[1] = mats[i%2]
