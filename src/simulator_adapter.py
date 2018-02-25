#!/usr/bin/env python2

import math
import re
import rospy
import tf.transformations
import tf2_ros as tf2

from iarc7_msgs.msg import (BoolStamped,
                            FlightControllerStatus,
                            Float64Stamped,
                            LandingGearContactsStamped,
                            Obstacle,
                            ObstacleArray,
                            OdometryArray,
                            OrientationAnglesStamped,
                            OrientationThrottleStamped,
                            PlanarThrottleStamped)
from geometry_msgs.msg import (PointStamped,
                               PoseStamped,
                               PoseWithCovarianceStamped,
                               Quaternion,
                               TransformStamped,
                               Vector3,
                               Vector3Stamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Range
from std_msgs.msg import (Float64,
                          Float32MultiArray,
                          MultiArrayDimension)
from iarc7_msgs.srv import Arm, ArmResponse

import tf2_geometry_msgs

def sim_camera_callback(msg, pub):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    out_msg = bridge.cv2_to_imgmsg(img, encoding='rgb8')
    out_msg.header = msg.header
    pub.publish(out_msg)

def sim_odom_callback(odom_msg):
    odom_msg.pose.pose.orientation = Quaternion()
    odom_msg.pose.pose.orientation.w = 1.0
    odom_msg.twist.twist.angular = Vector3()
    odom_msg.child_frame_id = 'level_quad'
    odom_pub.publish(odom_msg)

def sim_pose_callback(pose_msg):
    orientation_msg = OrientationAnglesStamped()
    orientation_msg.header.stamp = pose_msg.header.stamp

    orientation = pose_msg.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation.x,
                                                                 orientation.y,
                                                                 orientation.z,
                                                                 orientation.w))
    orientation_msg.data.roll = roll
    orientation_msg.data.pitch = -pitch
    orientation_msg.data.yaw = (2*math.pi - yaw) % (2*math.pi)

    orientation_pub.publish(orientation_msg)

    if publish_ground_truth_orientation:
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

    if publish_ground_truth_camera_localization:
        camera_pose_msg = PoseWithCovarianceStamped()
        camera_pose_msg.header.stamp = pose_msg.header.stamp
        camera_pose_msg.header.frame_id = 'map'
        camera_pose_msg.pose.pose.position = pose_msg.pose.position
        camera_pose_msg.pose.covariance[0*6+0] = 0.000001
        camera_pose_msg.pose.covariance[1*6+1] = 0.000001
        camera_pose_msg.pose.covariance[2*6+2] = 100.0
        camera_pose_pub.publish(camera_pose_msg)

    if publish_ground_truth_altitude:
        # TODO: Make this also publish the altimeter_reading topic
        # Publish altimeter_pose
        altimeter_pose = PoseWithCovarianceStamped()
        altimeter_pose.header.stamp = pose_msg.header.stamp
        altimeter_pose.pose.pose.position.z = pose_msg.pose.position.z
        altimeter_pose_pub.publish(altimeter_pose)
        short_range_altimeter_pose_pub.publish(altimeter_pose)

def sim_front_switch_callback(msg):
    switches.front = msg.data

def sim_back_switch_callback(msg):
    switches.back = msg.data

def sim_left_switch_callback(msg):
    switches.left = msg.data

def sim_right_switch_callback(msg):
    switches.right = msg.data

def control_direction_callback(direction_msg,planar_direction_msg):
    attitude_msg = Float32MultiArray()
    attitude_msg.layout.dim.append(MultiArrayDimension())
    attitude_msg.layout.dim[0].label = ''
    attitude_msg.layout.dim[0].size = 4
    attitude_msg.layout.dim[0].stride = 1
    attitude_msg.layout.data_offset = 0

    if fc_status.armed:
        thrust_percentage = direction_msg.throttle
        planar_thrust =[planar_direction_msg.front_throttle,
                        planar_direction_msg.back_throttle,
                        planar_direction_msg.left_throttle,
                        planar_direction_msg.right_throttle
                ]

        attitude_msg.data = [
                -direction_msg.data.roll,
                direction_msg.data.pitch,
                direction_msg.data.yaw,
                0.01
            ]
    else:
        thrust_percentage = 0.0
        attitude_msg.data = [0.0, 0.0, 0.0, 0.0]
        planar_thrust = [0.0, 0.0, 0.0, 0.0]

    quad_thrust_pub.publish(thrust_percentage)
    quad_attitude_pub.publish(attitude_msg)

    quad_front_thrust_pub.publish(planar_thrust[0])
    quad_back_thrust_pub.publish(planar_thrust[1])
    quad_left_thrust_pub.publish(planar_thrust[2])
    quad_right_thrust_pub.publish(planar_thrust[3])

def altimeter_callback(altitude_msg):
    _altimeter_callback(altitude_msg, 0.0009, altimeter_pose_pub)

def short_range_altimeter_callback(altitude_msg):
    _altimeter_callback(altitude_msg, 0.0001, short_range_altimeter_pose_pub)

def _altimeter_callback(altitude_msg, cov, pub):
    if (altitude_msg.range < altitude_msg.min_range
     or altitude_msg.range > altitude_msg.max_range):
        return

    try:
        transform = tf2_buffer.lookup_transform('level_quad',
                                                altitude_msg.header.frame_id,
                                                altitude_msg.header.stamp,
                                                rospy.Duration(1.0))
    except tf2.ExtrapolationException as ex:
        latest_tf = tf2_buffer.lookup_transform('level_quad',
                                                altitude_msg.header.frame_id,
                                                rospy.Time(0))
        if latest_tf.header.stamp < altitude_msg.header.stamp:
            # There's a message older than the one we're looking for, so the
            # exception must be for extrapolation into the future, which is a
            # problem
            raise
        else:
            rospy.logwarn(
                'Altimeter message received at time {0} before tf was available'
                .format(altitude_msg.header.stamp))
    else:
        altimeter_frame_point = PointStamped()
        altimeter_frame_point.point.x = altitude_msg.range
        altimeter_frame_point.header.stamp = altitude_msg.header.stamp
        altimeter_frame_point.header.frame_id = altitude_msg.header.frame_id

        transformed_point = tf2_geometry_msgs.do_transform_point(altimeter_frame_point,
                                                                 transform)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = altitude_msg.header.stamp
        pose_msg.header.frame_id = 'map'

        # The covariance is a 6x6 matrix (stored as an array), we want entry (2, 2)
        pose_msg.pose.covariance[2*6 + 2] = cov
        pose_msg.pose.pose.position.z = -transformed_point.point.z

        pub.publish(pose_msg)

def arm_service_handler(request):
    fc_status.armed = request.data
    return ArmResponse(success=True)

def roomba_odom_callback(msg, topic, data={}):
    if not 'cur_odoms' in data:
        data['cur_odoms'] = {}
    data['cur_odoms'][topic] = msg

    if 'last_time' in data:
        earliest_publish_time = (data['last_time']
                               + rospy.Duration(1.0/max_roomba_output_freq))
        if earliest_publish_time > rospy.Time.now():
            return

    data['last_time'] = rospy.Time.now()
    out_msg = OdometryArray()
    out_msg.data = data['cur_odoms'].values()
    roomba_pub.publish(out_msg)

def obstacle_odom_callback(msg, topic, data={}):
    if not 'cur_odoms' in data:
        data['cur_odoms'] = {}
    data['cur_odoms'][topic] = msg

    if 'last_time' in data:
        earliest_publish_time = (data['last_time']
                               + rospy.Duration(1.0/max_obstacle_output_freq))
        if earliest_publish_time > rospy.Time.now():
            return

    data['last_time'] = rospy.Time.now()
    out_msg = ObstacleArray()
    out_msg.header.stamp = msg.header.stamp
    for odom in data['cur_odoms'].values():
        obst = Obstacle()
        obst.header.stamp = msg.header.stamp
        obst.odom = odom
        obst.base_radius = 0.17
        obst.base_height = 0.07
        obst.pipe_radius = 0.05
        obst.pipe_height = 2.0
        out_msg.obstacles.append(obst)
    obstacle_pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('simulator_adapter')

    # ROSPARAM
    publish_ground_truth_orientation = rospy.get_param(
            '/sim/ground_truth_orientation', False)
    publish_ground_truth_localization = rospy.get_param(
            '/sim/ground_truth_localization', False)
    publish_ground_truth_altitude = rospy.get_param(
            '/sim/ground_truth_altitude', False)
    publish_ground_truth_camera_localization = rospy.get_param(
            '/sim/ground_truth_camera_localization', False)
    publish_ground_truth_roombas = rospy.get_param(
            '/sim/ground_truth_roombas', False)
    publish_ground_truth_obstacles = rospy.get_param(
            '/sim/ground_truth_obstacles', False)
    max_roomba_output_freq = rospy.get_param(
            '/sim/max_roomba_output_freq', float('Inf'))
    max_obstacle_output_freq = rospy.get_param(
            '/sim/max_obstacle_output_freq', float('Inf'))
    num_roombas = rospy.get_param('/sim/num_roombas', 0)
    num_obstacles = rospy.get_param('/sim/num_obstacles', 0)

    front_camera_on = bool(rospy.get_param(
            '/sim/front_camera_resolution', False))
    left_camera_on = bool(rospy.get_param(
            '/sim/left_camera_resolution', False))
    right_camera_on = bool(rospy.get_param(
            '/sim/right_camera_resolution', False))
    back_camera_on = bool(rospy.get_param(
            '/sim/back_camera_resolution', False))
    bottom_camera_on = bool(rospy.get_param(
            '/sim/bottom_camera_resolution', False))

    publish_prototype_uav = rospy.get_param(
            '/sim/prototype_uav', False)

    # MORSE SIDE COMMUNICATION

    # Subscribers
    rospy.Subscriber('/sim/quad/pose', PoseStamped, sim_pose_callback)
    if publish_ground_truth_localization:
        rospy.Subscriber('/sim/quad/odom', Odometry, sim_odom_callback)
    rospy.Subscriber('/sim/switch_front', BoolStamped, sim_front_switch_callback)
    rospy.Subscriber('/sim/switch_back', BoolStamped, sim_back_switch_callback)
    rospy.Subscriber('/sim/switch_left', BoolStamped, sim_left_switch_callback)
    rospy.Subscriber('/sim/switch_right', BoolStamped, sim_right_switch_callback)

    # Publishers
    quad_attitude_pub = rospy.Publisher('/sim/quad/attitude_controller',
                                        Float32MultiArray,
                                        queue_size=0)
    quad_thrust_pub = rospy.Publisher('/sim/quad/thrust_controller',
                                      Float64,
                                      queue_size=0)


    if publish_prototype_uav:
        quad_front_thrust_pub = rospy.Publisher('/sim/quad/front_thrust_controller',
                Float64,
                queue_size=0)

        quad_back_thrust_pub = rospy.Publisher('/sim/quad/back_thrust_controller',
                Float64,
                queue_size=0)

        quad_left_thrust_pub = rospy.Publisher('/sim/quad/left_thrust_controller',
                Float64,
                queue_size=0)

        quad_right_thrust_pub = rospy.Publisher('/sim/quad/right_thrust_controller',
                Float64,
                queue_size=0)

    if publish_ground_truth_roombas:
        for i in range(num_roombas):
            rospy.Subscriber('/sim/roomba{}/odom'.format(i),
                             Odometry,
                             roomba_odom_callback,
                             ('/sim/roomba{}/odom'.format(i),))
    if publish_ground_truth_obstacles:
        for i in range(num_obstacles):
            rospy.Subscriber('/sim/obstacle{}/odom'.format(i),
                             Odometry,
                             obstacle_odom_callback,
                             ('/sim/obstacle{}/odom'.format(i),))

    # PUBLIC SIDE COMMUNICATION

    # Subscribers
    rospy.Subscriber('uav_direction_command',
                     OrientationThrottleStamped,
                     PlanarThrottleStamped,
                     control_direction_callback)
    if not publish_ground_truth_altitude:
        # We aren't publishing the ground truth altitude, so get the altimeter
        # reading from the topic
        rospy.Subscriber('altimeter_reading', Range, altimeter_callback)
        rospy.Subscriber('short_distance_lidar',
                         Range,
                         short_range_altimeter_callback)

    # Publishers
    fc_battery_pub = rospy.Publisher('fc_battery', Float64Stamped, queue_size=0)
    motor_battery_pub = rospy.Publisher('motor_battery',
                                                Float64Stamped,
                                                queue_size=0)
    status_pub = rospy.Publisher('fc_status', FlightControllerStatus, queue_size=0)
    orientation_pub = rospy.Publisher('fc_orientation',
                                      OrientationAnglesStamped,
                                      queue_size=10)
    if publish_ground_truth_localization:
        odom_pub = rospy.Publisher('odometry/filtered', Odometry, queue_size=10)
    if publish_ground_truth_altitude:
        altimeter_reading_pub = rospy.Publisher('altimeter_reading',
                                                Range,
                                                queue_size=0)
        short_range_altimeter_reading_pub = rospy.Publisher(
                'short_distance_lidar',
                Range,
                queue_size=0)
    if publish_ground_truth_camera_localization:
        camera_pose_pub = rospy.Publisher('camera_localized_pose',
                                          PoseWithCovarianceStamped,
                                          queue_size=0)
    if publish_ground_truth_roombas:
        roomba_pub = rospy.Publisher('roombas',
                                     OdometryArray,
                                     queue_size=0)
    if publish_ground_truth_obstacles:
        obstacle_pub = rospy.Publisher('obstacles',
                                       ObstacleArray,
                                       queue_size=0)
    altimeter_pose_pub = rospy.Publisher('altimeter_pose',
                                         PoseWithCovarianceStamped,
                                         queue_size=0)
    short_range_altimeter_pose_pub = rospy.Publisher(
            'short_distance_lidar_pose',
            PoseWithCovarianceStamped,
            queue_size=0)
    switches_pub = rospy.Publisher('landing_gear_contact_switches',
                                   LandingGearContactsStamped,
                                   queue_size=0)

    # CAMERA STUFF
    if (front_camera_on
     or left_camera_on
     or right_camera_on
     or back_camera_on
     or bottom_camera_on):
        import cv_bridge
        bridge = cv_bridge.CvBridge()

    if front_camera_on:
        front_camera_pub = rospy.Publisher('/front_camera/rgb/image_raw',
                                           Image,
                                           queue_size=10)
        rospy.Subscriber('/sim/front_camera/image_raw',
                         Image,
                         lambda msg: sim_camera_callback(msg,
                                                         front_camera_pub))
    if left_camera_on:
        left_camera_pub = rospy.Publisher('/left_camera/rgb/image_raw',
                                           Image,
                                           queue_size=10)
        rospy.Subscriber('/sim/left_camera/image_raw',
                         Image,
                         lambda msg: sim_camera_callback(msg,
                                                         left_camera_pub))
    if right_camera_on:
        right_camera_pub = rospy.Publisher('/right_camera/rgb/image_raw',
                                           Image,
                                           queue_size=10)
        rospy.Subscriber('/sim/right_camera/image_raw',
                         Image,
                         lambda msg: sim_camera_callback(msg,
                                                         right_camera_pub))
    if back_camera_on:
        back_camera_pub = rospy.Publisher('/back_camera/rgb/image_raw',
                                           Image,
                                           queue_size=10)
        rospy.Subscriber('/sim/back_camera/image_raw',
                         Image,
                         lambda msg: sim_camera_callback(msg,
                                                         back_camera_pub))
    if bottom_camera_on:
        bottom_camera_pub = rospy.Publisher('/bottom_camera/rgb/image_raw',
                                           Image,
                                           queue_size=10)
        rospy.Subscriber('/sim/bottom_camera/image_raw',
                         Image,
                         lambda msg: sim_camera_callback(msg,
                                                         bottom_camera_pub))

    # Services
    rospy.Service('uav_arm', Arm, arm_service_handler)

    # TF OBJECTS
    tf2_broadcaster = tf2.TransformBroadcaster()
    tf2_buffer = tf2.Buffer()
    tf2_listener = tf2.TransformListener(tf2_buffer)

    # RATE CONTROL
    frequency = rospy.get_param('frequency', 50)
    rate = rospy.Rate(frequency)

    # MESSAGES
    fc_status = FlightControllerStatus()
    fc_status.armed = False
    fc_status.auto_pilot = True
    fc_status.failsafe = False

    switches = LandingGearContactsStamped()
    switches.front = False
    switches.back = False
    switches.left = False
    switches.right = False

    # MAIN LOOP
    while not rospy.is_shutdown():
        battery_msg = Float64Stamped()
        battery_msg.data = 0.0
        battery_msg.header.stamp = rospy.Time.now()
        fc_battery_pub.publish(battery_msg)

        battery_msg.data = 12.6
        motor_battery_pub.publish(battery_msg)

        fc_status.header.stamp = rospy.get_rostime()
        status_pub.publish(fc_status)

        switches.header.stamp = rospy.get_rostime()
        switches_pub.publish(switches)

        rate.sleep()
