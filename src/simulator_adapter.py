#!/usr/bin/env python2
import rospy

from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import Float32, Float64, Float32MultiArray, Header, MultiArrayDimension
from iarc7_msgs.msg import FlightControllerStatus, Float64Stamped, OrientationThrottleStamped

import tf2_ros as tf2

def sim_accel_callback(twist_msg):
    accel_msg = Vector3Stamped()
    accel_msg.header = Header()
    accel_msg.header.stamp = twist_msg.header.stamp
    accel_msg.header.frame_id = 'quad'
    accel_msg.vector = twist_msg.twist.linear

    accel_pub.publish(accel_msg)

def sim_pose_callback(pose_msg):
    transform_msg = TransformStamped()
    transform_msg.header.stamp = pose_msg.header.stamp
    transform_msg.header.frame_id = 'level_quad'
    transform_msg.child_frame_id = 'quad'
    transform_msg.transform.rotation = pose_msg.pose.orientation

    tf2_broadcaster.sendTransform(transform_msg)

    if publish_ground_truth_localization:
        transform_msg = TransformStamped()
        transform_msg.header.stamp = pose_msg.header.stamp
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = 'level_quad'
        transform_msg.transform.translation = pose_msg.pose.position
        transform_msg.transform.rotation.w = 1

        tf2_broadcaster.sendTransform(transform_msg)

    if publish_ground_truth_altitude:
        altitude_msg = Float64Stamped()
        altitude_msg.header.stamp = pose_msg.header.stamp
        altitude_msg.header.frame_id = 'map'
        altitude_msg.data = pose_msg.pose.position.z

        altitude_pub.publish(altitude_msg)

def control_direction_callback(direction_msg):
    # Incoming range is [0, 100], outgoing is [0, 1]
    quad_thrust_pub.publish(direction_msg.throttle * 0.01)

    attitude_msg = Float32MultiArray()
    attitude_msg.layout.dim.append(MultiArrayDimension())
    attitude_msg.layout.dim[0].label = ''
    attitude_msg.layout.dim[0].size = 4
    attitude_msg.layout.dim[0].stride = 1
    attitude_msg.layout.data_offset = 0
    attitude_msg.data = [
            direction_msg.data.roll,
            -direction_msg.data.pitch,
            -direction_msg.data.yaw,
            0.01
        ]

    quad_attitude_pub.publish(attitude_msg)

if __name__ == '__main__':
    rospy.init_node('simulator_adapter')

    # ROSPARAM
    publish_ground_truth_localization = rospy.get_param(
            '/sim/ground_truth_localization', False)
    publish_ground_truth_altitude = rospy.get_param(
            '/sim/ground_truth_altitude', False)

    # MORSE SIDE COMMUNICATION

    # Subscribers
    rospy.Subscriber('/sim/pose', PoseStamped, sim_pose_callback)
    rospy.Subscriber('/sim/quad_accel', TwistStamped, sim_accel_callback)

    # Publishers
    quad_attitude_pub = rospy.Publisher('/sim/quad_attitude_controller',
                                        Float32MultiArray,
                                        queue_size=0)
    quad_thrust_pub = rospy.Publisher('/sim/quad_thrust_controller',
                                      Float64,
                                      queue_size=0)

    # PUBLIC SIDE COMMUNICATION

    # Subscribers
    rospy.Subscriber('uav_direction_command',
                     OrientationThrottleStamped,
                     control_direction_callback)

    # Publishers
    accel_pub = rospy.Publisher('acceleration', Vector3Stamped, queue_size=0)
    battery_pub = rospy.Publisher('fc_battery', Float32, queue_size=0)
    status_pub = rospy.Publisher('fc_status', FlightControllerStatus, queue_size=0)
    if publish_ground_truth_altitude:
        altitude_pub = rospy.Publisher('altitude', Float64Stamped, queue_size=0)

    # TF OBJECTS
    tf2_broadcaster = tf2.TransformBroadcaster()

    # RATE CONTROL
    frequency = rospy.get_param('frequency', 50)
    rate = rospy.Rate(frequency)

    # MAIN LOOP
    while not rospy.is_shutdown():
        battery_pub.publish(12.6)

        status_msg = FlightControllerStatus()
        status_msg.header = Header()
        status_msg.header.stamp = rospy.get_rostime()
        status_msg.armed = True
        status_msg.auto_pilot = True
        status_msg.failsafe = False
        status_pub.publish(status_msg)

        rate.sleep()
