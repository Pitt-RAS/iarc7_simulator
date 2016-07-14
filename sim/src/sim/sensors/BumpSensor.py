import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

class BumpSensor(morse.core.sensor.Sensor):

    _name = "BumpSensor"
    _short_desc = "Roomba bumper based on the Radar sensor"

    add_data('positive', False, 'bool', 'True if there is an object in the radar cone')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        logger.info('Component initialized')

    @service
    def get_positive(self):
        logger.info("%s is %s"%(self.name, 'positive' if self.local_data['positive'] else 'negative'))
        return self.local_data['positive']

    def default_action(self):
        self.local_data['positive'] = self.bge_object.sensors['Radar'].positive
