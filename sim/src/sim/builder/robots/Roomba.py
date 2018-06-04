import math
from morse.builder import *
from sim.builder.sensors.BumpSensor import BumpSensor
from sim.builder.sensors.OdometryAbsolutePose import OdometryAbsolutePose

class Roomba(GroundRobot):
    """
    A template robot model for Roomba, with a motion controller and a pose sensor.
    """

    ROOMBA_RADIUS = 0.165
    ROOMBA_HEIGHT = 0.095

    # Angle occupied by one sensor in radians
    BUMP_SENSOR_ANGLE = 0.489957 / 2
    NUM_BUMP_SENSORS = 12

    def __init__(self, filename=None, name=None, debug=False, publish_tf=True):
        # Roomba.blend is located in the data/robots directory
        filename = filename or 'sim/robots/Roomba.blend'
        name = name or 'roomba'

        GroundRobot.__init__(self, filename, name)
        self.properties(classpath = "sim.robots.Roomba.Roomba")

        ###################################
        # Actuators
        ###################################

        # (v,w) motion controller
        # Check here the other available actuators:
        # http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
        self.motion = MotionVW()
        self.motion.add_stream('ros', topic='/sim/{}/cmd_vel'.format(name))
        self.append(self.motion)

        # Optionally allow to move the robot with the keyboard
        if debug:
            keyboard = Keyboard()
            keyboard.properties(ControlType = 'Position')
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        frame_prefix = '' if publish_tf else 'sim/'
        self.odom = OdometryAbsolutePose()
        self.odom.add_stream('ros',
                             'morse.middleware.ros.odometry.OdometryPublisher',
                             topic='/sim/{}/odom'.format(name),
                             frame_id='map',
                             child_frame_id=frame_prefix + '{}/base_link'.format(name))
        self.append(self.odom)

        angle = -float(Roomba.NUM_BUMP_SENSORS - 1) / 2 * Roomba.BUMP_SENSOR_ANGLE
        sensor_radius = Roomba.ROOMBA_RADIUS + 0.01
        for i in range(Roomba.NUM_BUMP_SENSORS):
            next_bumper = BumpSensor()
            next_bumper.translate(sensor_radius * math.cos(angle),
                                  sensor_radius * math.sin(angle),
                                  Roomba.ROOMBA_HEIGHT / 2.0)
            next_bumper.rotate(0, 0, angle)
            next_bumper.add_stream(
                    'ros',
                    'sim.middleware.ros.bump_sensor.BumpSensorPublisher',
                    topic='/sim/{}/bumper{}'.format(name, i))
            self.append(next_bumper)
            angle += Roomba.BUMP_SENSOR_ANGLE
