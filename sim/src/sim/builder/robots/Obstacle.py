import math
from morse.builder import *
from sim.builder.sensors.BumpSensor import BumpSensor

# Angle occupied by one sensor in radians
BUMP_SENSOR_ANGLE = 0.489957
NUM_BUMP_SENSORS = 6

BASE_RADIUS = 0.15
BASE_HEIGHT = 0.1

class Obstacle(GroundRobot):
    """
    A template robot model for Obstacle, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = False):

        # Obstacle.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'sim/robots/Obstacle.blend', name)
        self.properties(classpath = "sim.robots.Obstacle.Obstacle")

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

        angle = -float(NUM_BUMP_SENSORS - 1) / 2 * BUMP_SENSOR_ANGLE
        sensor_radius = BASE_RADIUS + 0.01
        for i in range(6):
            next_bumper = BumpSensor('Bumper%i'%i)
            next_bumper.translate(sensor_radius * math.cos(angle),
                                  sensor_radius * math.sin(angle),
                                  BASE_HEIGHT / 2.0)
            next_bumper.rotate(0, 0, angle)
            self.append(next_bumper)
            angle += BUMP_SENSOR_ANGLE
