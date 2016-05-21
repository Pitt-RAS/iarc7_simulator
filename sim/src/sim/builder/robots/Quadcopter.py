from morse.builder import *

class Quadcopter(Robot):
    """
    A template robot model for Quadcopter, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

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
        self.append(self.motion)

        # Optionally allow to move the robot with the keyboard
        if debug:
            keyboard = Keyboard()
            keyboard.properties(ControlType = 'Position')
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.append(self.pose)

        self.velocity = Velocity()
        self.append(self.velocity)
