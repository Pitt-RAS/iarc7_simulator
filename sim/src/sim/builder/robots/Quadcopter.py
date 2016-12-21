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
                 create_front_camera=True,
                 create_left_camera=True,
                 create_right_camera=True,
                 create_bottom_camera=True):

        # Quadcopter.blend is located in the data/robots directory
        Robot.__init__(self, 'sim/robots/Quadcopter.blend', name)
        self.properties(classpath = "sim.robots.Quadcopter.Quadcopter")

        ###################################
        # Actuators
        ###################################

        self.motion = RotorcraftAttitude()
        self.motion.properties(RollPitchPgain = 1000.0,
                               RollPitchDgain = 400.0,
                               YawPgain = 3,
                               YawDgain = 50,
                               ThrustFactor = 1,
                               YawRateControl = True
                               )
        self.motion.add_stream('ros', topic='/sim/quad_attitude_controller')
        self.append(self.motion)

        self.thrust = Thrust()
        self.thrust.properties(NumberOfProps = 4,
                               PropDiameter = 0.2,
                               PropPitch = 0.1,
                               PropMaxSpeed = 8214 / 60.0)
        self.thrust.add_stream('ros', 'sim.middleware.ros.thrust.ThrustReader', topic='/sim/quad_thrust_controller')
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
        self.lidar.add_stream('ros', 'sim.middleware.ros.lidarlite.LidarLitePublisher', topic='/altitude_raw')
        self.append(self.lidar)

        self.velocity = Velocity()
        self.append(self.velocity)

        if create_bottom_camera:
            self.bottom_camera = VideoCamera()
            self.bottom_camera.properties(
                    cam_width=512,
                    cam_height=512
                    )
            self.bottom_camera.translate(z=-0.2)
            self.bottom_camera.rotate(y=-math.pi/2)
            self.bottom_camera.add_stream('ros', topic='/bottom_image_raw')
            self.append(self.bottom_camera)

        if create_front_camera:
            self.front_camera = VideoCamera()
            self.front_camera.properties(
                    cam_width=512,
                    cam_height=512
                    )
            self.front_camera.translate(x=0.3)
            self.front_camera.rotate(y=-math.pi/6)
            self.front_camera.add_stream('ros', topic='/front_image_raw')
            self.append(self.front_camera)

        if create_right_camera:
            self.right_camera = VideoCamera()
            self.right_camera.properties(
                    cam_width=512,
                    cam_height=512
                    )
            self.right_camera.translate(y=0.2)
            self.right_camera.rotate(y=-math.pi/2, z=-math.pi/4)
            self.right_camera.add_stream('ros', topic='/right_image_raw')
            self.append(self.right_camera)

        if create_left_camera:
            self.left_camera = VideoCamera()
            self.left_camera.properties(
                    cam_width=512,
                    cam_height=512
                    )
            self.left_camera.translate(y=-0.2)
            self.left_camera.rotate(y=-math.pi/2, z=math.pi/4)
            self.left_camera.add_stream('ros', topic='/left_image_raw')
            self.append(self.left_camera)
