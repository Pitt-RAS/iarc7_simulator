import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

import mathutils

class LidarLite(morse.core.sensor.Sensor):
    _name = "LidarLite"
    _short_desc = "Lidar Lite v2 Rangefinder (pointing in the +x direction)"

    add_data('range', float('NaN'), 'float', 'Range reading')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        morse.core.sensor.Sensor.__init__(self, obj, parent)

    def default_action(self):
        obj = self.bge_object
        direction = self.position_3d.matrix * mathutils.Vector([1, 0, 0])
        ray_cast = obj.rayCast(direction, None, 100)

        if ray_cast[0] is not None:
            self.local_data['range'] = round(obj.getDistanceTo(ray_cast[1]), 2)
        else:
            self.local_data['range'] = float('NaN')
