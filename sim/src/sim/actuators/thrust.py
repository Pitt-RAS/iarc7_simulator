import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator

from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property

import math

AIR_DENSITY = 1.225

class Thrust(morse.core.actuator.Actuator):
    _name = "Thrust"
    _short_desc = "Thrust actuator for a quadcopter"

    add_data('throttle', 0, 'float', 'The throttle, between 0.0 and 1.0 inclusive')

    add_property('_number_of_props', 4, 'NumberOfProps', 'int',
                 'Number of propellors')
    add_property('_prop_diameter', 0.1, 'PropDiameter', 'float',
                 'Diameter of a propellor (in m)')
    add_property('_prop_pitch', 0.1, 'PropPitch', 'float',
                 'Pitch of a propellor (in m)')
    add_property('_prop_max_speed', 100.0, 'PropMaxSpeed', 'float',
                 'Max speed of a propellor (in revs / s)')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

    def default_action(self):
        prop_area = math.pi * (self._prop_diameter / 2)**2
        volume_per_rev = self._prop_pitch * prop_area
        mass_per_rev = AIR_DENSITY * volume_per_rev

        max_thrust = (self._prop_max_speed * self._prop_pitch + 2) * mass_per_rev * self._prop_max_speed

        z_velocity = self.robot_parent.bge_object.linearVelocity[2]
        prop_speed = self.local_data['throttle'] * self._prop_max_speed
        thrust = (prop_speed * self._prop_pitch - z_velocity) * mass_per_rev * prop_speed
        thrust = min(max_thrust, max(thrust, 0))
        thrust *= self._number_of_props

        self.robot_parent.bge_object.applyForce((0, 0, thrust), True)
