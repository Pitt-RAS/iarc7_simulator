#! /usr/bin/env morseexec

import math
from morse.builder import Clock, Environment, FakeRobot
from morse.core.morse_time import TimeStrategies
from sim.builder.robots import TargetRoomba, Obstacle, Quadcopter, X525

# Get ROS parameters
import rospy
floor_material_name = rospy.get_param('sim/floor')
front_camera_resolution = rospy.get_param('sim/front_camera_resolution', None)
left_camera_resolution = rospy.get_param('sim/left_camera_resolution', None)
right_camera_resolution = rospy.get_param('sim/right_camera_resolution', None)
back_camera_resolution = rospy.get_param('sim/back_camera_resolution', None)
bottom_camera_resolution = rospy.get_param('sim/bottom_camera_resolution', None)
create_teleport_actuator = rospy.get_param('sim/create_teleport_actuator', False)
prototype_uav = rospy.get_param('sim/prototype_uav', True)
no_drone = rospy.get_param('sim/no_drone', False)

publish_ground_truth_roombas = rospy.get_param(
        '/sim/ground_truth_roombas', False)
publish_ground_truth_obstacles = rospy.get_param(
        '/sim/ground_truth_obstacles', False)

num_targets = rospy.get_param('sim/num_roombas')
num_obstacles = rospy.get_param('sim/num_obstacles')
# Place roombas
roomba_placement_radius = 1
for i in range(num_targets):
    angle = 2*math.pi / num_targets * i
    robot = TargetRoomba('roomba%i'%i, publish_tf=publish_ground_truth_roombas)
    robot.translate(roomba_placement_radius * math.cos(angle),
            roomba_placement_radius * math.sin(angle),
            0)
    robot.rotate(0, 0, angle)

# Place obstacles
obstacle_placement_radius = 5
for i in range(num_obstacles):
    angle = 2*math.pi / num_obstacles * i
    robot = Obstacle('obstacle%i'%i, publish_tf=publish_ground_truth_obstacles)
    robot.translate(obstacle_placement_radius * -math.sin(angle),
            obstacle_placement_radius * math.cos(angle),
            0)
    robot.rotate(0, 0, angle)

if not no_drone:
    # Place drone
    if not prototype_uav:
        robot = Quadcopter('Quadcopter',
                        front_camera_resolution=front_camera_resolution,
                        left_camera_resolution=left_camera_resolution,
                        right_camera_resolution=right_camera_resolution,
                        back_camera_resolution=back_camera_resolution,
                        bottom_camera_resolution=bottom_camera_resolution,
                        create_teleport_actuator=create_teleport_actuator)
        robot.translate(0, 0, 0.2)

    elif prototype_uav:
        robot = X525('X525',
                        front_camera_resolution=front_camera_resolution,
                        left_camera_resolution=left_camera_resolution,
                        right_camera_resolution=right_camera_resolution,
                        back_camera_resolution=back_camera_resolution,
                        bottom_camera_resolution=bottom_camera_resolution,
                        create_teleport_actuator=create_teleport_actuator)
        robot.translate(0, 0, 0.035)

# Add /clock publisher
fake_robot = FakeRobot()
clock = Clock()
#clock.frequency(101)
clock.add_interface('ros', topic='/clock')
fake_robot.append(clock)

# set 'fastmode' to True to switch to wireframe mode
env = Environment('environment.blend', fastmode = False)

# These have to be set for the imu (really the magnetometer in the imu) to work.
env.properties(latitude=0.0, longitude=0.0, altitude=0.0)

#env.simulator_frequency(101)
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
