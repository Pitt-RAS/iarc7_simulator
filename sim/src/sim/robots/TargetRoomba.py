import logging; logger = logging.getLogger("morse." + __name__)
from sim.robots.Roomba import Roomba

class TargetRoomba(Roomba):
    """ 
    Class definition for the TargetRoomba robot.
    """

    _name = 'TargetRoomba robot'

    def __init__(self, obj, parent=None):
        """ Constructor method

        Receives the reference to the Blender object.
        Optionally it gets the name of the object's parent,
        but that information is not currently used for a robot.
        """

        logger.info('%s initialization' % obj.name)
        Roomba.__init__(self, obj, parent)

        # Do here robot specific initializations
        logger.info('Component initialized')

    def default_action(self):
        """ Main loop of the robot
        """

        # This is usually not used (responsibility of the actuators
        # and sensors). But you can add here robot-level actions.
        pass
