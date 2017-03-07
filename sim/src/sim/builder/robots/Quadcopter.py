from morse.builder import *
from sim.builder.actuators.thrust import Thrust
from sim.builder.sensors.LidarLite import LidarLite

import math

class Quadcopter(Robot):
    """
    A template robot model for Quadcopter, with a motion controller and a pose
    sensor.
    """
    def __init__(self, name=None,
                 front_camera_resolution=None,
                 left_camera_resolution=None,
                 right_camera_resolution=None,
                 bottom_camera_resolution=None):

        # Quadcopter.blend is located in the data/robots directory
        Robot.__init__(self, 'sim/robots/Quadcopter.blend', name)
        self.properties(classpath = "sim.robots.Quadcopter.Quadcopter")

        ###################################
        # Actuators
        ###################################

        self.motion = RotorcraftAttitude()
        self.motion.properties(RollPitchPgain = 5000.0,
                               RollPitchDgain = 800.0,
                               YawPgain = 10000.0,
                               YawDgain = 1000.0,
                               ThrustFactor = 1.0,
                               YawRateControl = True
                               )
        self.motion.add_stream('ros', topic='/sim/quad_attitude_controller')
        self.append(self.motion)

        self.thrust = Thrust()
        self.thrust.properties(NumberOfProps = 4,
                               PropDiameter = 0.25,
                               PropPitch = 0.1,
                               PropMaxSpeed = 12214 / 60.0)
        self.thrust.add_stream('ros',
                               'sim.middleware.ros.thrust.ThrustReader',
                               topic='/sim/quad_thrust_controller')
        self.append(self.thrust)

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.pose.add_stream('ros', topic='/sim/pose')
        self.append(self.pose)

        self.accel = Accelerometer()
        self.accel.add_stream('ros', topic='/sim/quad_accel')
        self.append(self.accel)

        self.lidar = LidarLite()
        self.lidar.translate(z=-0.1)
        self.lidar.rotate(y=math.pi/2)
        self.lidar.add_stream('ros',
                              'sim.middleware.ros.lidarlite.LidarLitePublisher',
                              topic='altimeter_reading',
                              frame='lidarlite')
        self.append(self.lidar)

        self.velocity = Velocity()
        self.append(self.velocity)

        if bottom_camera_resolution:
            self.bottom_camera = VideoCamera()
            self.bottom_camera.properties(
                    cam_width=bottom_camera_resolution[0],
                    cam_height=bottom_camera_resolution[1]
                    )
            self.bottom_camera.translate(z=-0.2)
            self.bottom_camera.rotate(y=-math.pi/2)
            self.bottom_camera.add_stream('ros', topic='/bottom_image_raw')
            self.append(self.bottom_camera)

        if front_camera_resolution:
            self.front_camera = VideoCamera()
            self.front_camera.properties(
                    cam_width=front_camera_resolution[0],
                    cam_height=front_camera_resolution[1]
                    )
            self.front_camera.translate(x=0.3)
            self.front_camera.rotate(y=-math.pi/6)
            self.front_camera.add_stream('ros', topic='/front_image_raw')
            self.append(self.front_camera)

        if right_camera_resolution:
            self.right_camera = VideoCamera()
            self.right_camera.properties(
                    cam_width=right_camera_resolution[0],
                    cam_height=right_camera_resolution[1]
                    )
            self.right_camera.translate(y=0.2)
            self.right_camera.rotate(y=-math.pi/2, z=-math.pi/4)
            self.right_camera.add_stream('ros', topic='/right_image_raw')
            self.append(self.right_camera)

        if left_camera_resolution:
            self.left_camera = VideoCamera()
            self.left_camera.properties(
                    cam_width=left_camera_resolution[0],
                    cam_height=left_camera_resolution[1]
                    )
            self.left_camera.translate(y=-0.2)
            self.left_camera.rotate(y=-math.pi/2, z=math.pi/4)
            self.left_camera.add_stream('ros', topic='/left_image_raw')
            self.append(self.left_camera)
