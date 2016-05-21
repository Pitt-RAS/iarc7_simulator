import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot
from morse.core.services import service

class Quadcopter(morse.core.robot.Robot):
    """ 
    Class definition for the Quadcopter robot.
    """

    _name = 'Quadcopter robot'

    def __init__(self, obj, parent=None):
        """ Constructor method

        Receives the reference to the Blender object.
        Optionally it gets the name of the object's parent,
        but that information is not currently used for a robot.
        """

        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        # Do here robot specific initializations
        logger.info('Component initialized')
        self.thrust = 9.8

    @service
    def set_thrust(self, thrust):
        self.thrust = thrust

    def default_action(self):
        """ Main loop of the robot
        """

        # This is usually not used (responsibility of the actuators
        # and sensors). But you can add here robot-level actions.
        self.bge_object.applyForce((0, 0, self.thrust), True)
