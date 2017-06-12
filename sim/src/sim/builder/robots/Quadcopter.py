from morse.builder import *
from sim.builder.actuators.thrust import Thrust
from sim.builder.sensors.LidarLite import LidarLite
from sim.builder.sensors.ContactSwitch import ContactSwitch

import math

class Quadcopter(Robot):

    QUAD_CENTER_HEIGHT = 0.21349
    QUAD_FOOT_SQUARE_TRANSLATION = 0.21

    """
    A template robot model for Quadcopter, with a motion controller and a pose
    sensor.
    """
    def __init__(self, name=None,
                 front_camera_resolution=None,
                 left_camera_resolution=None,
                 right_camera_resolution=None,
                 back_camera_resolution=None,
                 bottom_camera_resolution=None,
                 create_teleport_actuator=False):

        # Quadcopter.blend is located in the data/robots directory
        Robot.__init__(self, 'sim/robots/Quadcopter.blend', name)
        self.properties(classpath = "sim.robots.Quadcopter.Quadcopter")

        ###################################
        # Actuators
        ###################################

        if create_teleport_actuator:
            self.teleport = Teleport()
            self.teleport.add_stream('ros', topic='/sim/quad/teleport')
            self.append(self.teleport)
        else:
            self.motion = RotorcraftAttitude()
            self.motion.properties(RollPitchPgain = 5000.0,
                                   RollPitchDgain = 800.0,
                                   YawPgain = 10000.0,
                                   YawDgain = 1000.0,
                                   ThrustFactor = 1.0,
                                   YawRateControl = True
                                   )
            self.motion.add_stream('ros', topic='/sim/quad/attitude_controller')
            self.append(self.motion)

            self.thrust = Thrust()
            self.thrust.properties(NumberOfProps = 4,
                                   PropDiameter = 0.25,
                                   PropPitch = 0.1,
                                   PropMaxSpeed = 12214 / 60.0)
            self.thrust.add_stream('ros',
                                   'sim.middleware.ros.thrust.ThrustReader',
                                   topic='/sim/quad/thrust_controller')
            self.append(self.thrust)

        ###################################
        # Sensors
        ###################################

        self.odom = Odometry()
        self.odom.level('integrated')
        self.odom.add_stream('ros',
                             topic='/sim/quad/odom',
                             frame_id='map',
                             child_frame_id='sim/quad/odom')
        self.append(self.odom)

        self.pose = Pose()
        self.pose.add_stream('ros', topic='/sim/quad/pose')
        self.append(self.pose)

        self.imu = IMU()
        self.imu.add_stream('ros', topic='/fc_imu', frame_id='quad')
        self.append(self.imu)

        self.lidar = LidarLite()
        self.lidar.properties(min_range=0.0, max_range=100.0)
        self.lidar.translate(z=-0.1)
        self.lidar.rotate(y=math.pi/2)
        self.lidar.add_stream('ros',
                              'sim.middleware.ros.lidarlite.LidarLitePublisher',
                              topic='altimeter_reading',
                              frame='lidarlite')
        self.append(self.lidar)

        self.sharpir = LidarLite()
        self.sharpir.properties(min_range=0.0, max_range=0.8)
        self.sharpir.translate(z=-0.01, x=0.1)
        self.sharpir.rotate(y=math.pi/2)
        self.sharpir.add_stream('ros',
                                'sim.middleware.ros.lidarlite.LidarLitePublisher',
                                topic='short_distance_lidar',
                                frame='sharp_ir')
        self.append(self.sharpir)

        self.laserscanner = LaserSensorWithArc()
        self.laserscanner.properties(laser_range=6.0)
        self.laserscanner.properties(resolution=1.0)
        self.laserscanner.properties(scan_window=360.0)
        self.laserscanner.frequency(10.0)
        self.laserscanner.add_stream('ros', topic='scan')
        self.append(self.laserscanner)

        for translate in (((0, 1), 'front'),
                          ((0, -1), 'back'),
                          ((1, 0), 'right'),
                          ((-1, 0), 'left')):
            next_switch = ContactSwitch()
            next_switch.translate(Quadcopter.QUAD_FOOT_SQUARE_TRANSLATION * translate[0][1],
                                  -Quadcopter.QUAD_FOOT_SQUARE_TRANSLATION * translate[0][0],
                                  -Quadcopter.QUAD_CENTER_HEIGHT)
            next_switch.rotate(0, math.pi/2, 0)
            next_switch.add_stream(
                    'ros',
                    'sim.middleware.ros.bump_sensor.BumpSensorPublisher',
                    topic='/sim/switch_{}'.format(translate[1]))
            self.append(next_switch)

        self.velocity = Velocity()
        self.append(self.velocity)

        if bottom_camera_resolution:
            self.bottom_camera = VideoCamera()
            self.bottom_camera.properties(
                    cam_width=bottom_camera_resolution[0],
                    cam_height=bottom_camera_resolution[1]
                    )
            self.bottom_camera.translate(z=-0.2)
            self.bottom_camera.rotation_euler = (math.pi, 0, -math.pi/2)
            self.bottom_camera.frequency(bottom_camera_resolution[2])
            self.bottom_camera.add_stream('ros',
                                          topic='/bottom_image_raw',
                                          frame_id='bottom_camera_optical',
                                          parent_frame_id='quad')
            self.append(self.bottom_camera)

        if front_camera_resolution:
            self.front_camera = VideoCamera()
            self.front_camera.properties(
                    cam_width=front_camera_resolution[0],
                    cam_height=front_camera_resolution[1]
                    )
            self.front_camera.translate(x=0.2)
            self.front_camera.frequency(front_camera_resolution[2])
            self.front_camera.add_stream('ros',
                                         topic='/front_image_raw',
                                         frame_id='front_camera_optical',
                                         parent_frame_id='quad')
            self.append(self.front_camera)

        if left_camera_resolution:
            self.left_camera = VideoCamera()
            self.left_camera.properties(
                    cam_width=left_camera_resolution[0],
                    cam_height=left_camera_resolution[1]
                    )
            self.left_camera.translate(y=0.2)
            self.left_camera.rotate(x=math.pi/2)
            self.left_camera.frequency(left_camera_resolution[2])
            self.left_camera.add_stream('ros',
                                        topic='/left_image_raw',
                                        frame_id='left_camera_optical',
                                        parent_frame_id='quad')
            self.append(self.left_camera)

        if right_camera_resolution:
            self.right_camera = VideoCamera()
            self.right_camera.properties(
                    cam_width=right_camera_resolution[0],
                    cam_height=right_camera_resolution[1]
                    )
            self.right_camera.translate(y=-0.2)
            self.right_camera.rotate(x=-math.pi/2)
            self.right_camera.frequency(right_camera_resolution[2])
            self.right_camera.add_stream('ros',
                                         topic='/right_image_raw',
                                         frame_id='right_camera_optical',
                                         parent_frame_id='quad')
            self.append(self.right_camera)

        if back_camera_resolution:
            self.back_camera = VideoCamera()
            self.back_camera.properties(
                    cam_width=back_camera_resolution[0],
                    cam_height=back_camera_resolution[1]
                    )
            self.back_camera.translate(x=-0.2)
            self.back_camera.rotate(x=math.pi)
            self.back_camera.add_stream('ros',
                                         topic='/back_image_raw',
                                         frame_id='back_camera_optical',
                                         parent_frame_id='quad')
            self.append(self.back_camera)
