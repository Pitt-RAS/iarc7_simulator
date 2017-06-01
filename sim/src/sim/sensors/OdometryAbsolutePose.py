import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

from morse.sensors.odometry import IntegratedOdometry

class OdometryAbsolutePose(IntegratedOdometry):
    _name = "OdometryAbsolutePose"

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        IntegratedOdometry.__init__(self, obj, parent)

        self.original_pos = morse.helpers.transformation.Transformation3d(None)
        self.previous_pos = self.original_pos.transformation3d_with(self.position_3d)

        logger.info('Component initialized')
