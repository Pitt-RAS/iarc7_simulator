from morse.builder import *
from sim.builder.sensors.BumpSensor import BumpSensor
from sim.builder.sensors.TopTouchSensor import TopTouchSensor

class Roomba(GroundRobot):
    """
    A template robot model for Roomba, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = False):

        # Roomba.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'sim/robots/Roomba.blend', name)
        self.properties(classpath = "sim.robots.Roomba.Roomba")

        ###################################
        # Actuators
        ###################################

        # (v,w) motion controller
        # Check here the other available actuators:
        # http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
        self.motion = MotionVW()
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

        self.FrontBumper = BumpSensor('FrontBumper')
        self.FrontBumper.translate(0.15, 0, 0.05)
        self.append(self.FrontBumper)

        self.LeftBumper = BumpSensor('LeftBumper')
        self.LeftBumper.translate(0.15 / 2, 0.15 * math.sqrt(3) / 2, 0.05)
        self.LeftBumper.rotate(0, 0, math.pi / 3)
        self.append(self.LeftBumper)

        self.RightBumper = BumpSensor('RightBumper')
        self.RightBumper.translate(0.15 / 2, -0.15 * math.sqrt(3) / 2, 0.05)
        self.RightBumper.rotate(0, 0, -math.pi / 3)
        self.append(self.RightBumper)

        self.top_sensor = TopTouchSensor('TopTouchSensor')
        self.top_sensor.translate(0, 0, 0.1)
        self.append(self.top_sensor)
