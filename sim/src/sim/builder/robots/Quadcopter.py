from morse.builder import *
from sim.builder.actuators.thrust import Thrust

class Quadcopter(Robot):
    """
    A template robot model for Quadcopter, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None):

        # Quadcopter.blend is located in the data/robots directory
        Robot.__init__(self, 'sim/robots/Quadcopter.blend', name)
        self.properties(classpath = "sim.robots.Quadcopter.Quadcopter")

        ###################################
        # Actuators
        ###################################

        self.motion = RotorcraftAttitude()
        self.motion.properties(RollPitchPgain = 100,
                               RollPitchDgain = 50,
                               YawPgain = 3,
                               YawDgain = 50,
                               ThrustFactor = 1,
                               YawRateControl = False
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

        self.velocity = Velocity()
        self.append(self.velocity)
