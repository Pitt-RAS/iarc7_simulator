import math

# all units in mks

NUMBER_OF_PROPS = 4

PROP_DIAMETER = 0.2
PROP_PITCH = 0.1
PROP_MAX_SPEED = 8214 / 60.0 # ideally 14985 / 60.0 # revs / s
AIR_DENSITY = 1.225

PROP_AREA = math.pi * (PROP_DIAMETER / 2)**2
VOLUME_PER_REV = PROP_PITCH * PROP_AREA
MASS_PER_REV = AIR_DENSITY * VOLUME_PER_REV

MAX_THRUST = (PROP_MAX_SPEED * PROP_PITCH + 2) * MASS_PER_REV * PROP_MAX_SPEED

class Quadcopter:
    def __init__(self, owner):
        self._owner = owner

        self._update_velocity(self._owner.velocity.get())
        self._owner.velocity.subscribe(lambda data: self._update_velocity(data))
        self._update_pose(self._owner.pose.get())
        self._owner.pose.subscribe(lambda data: self._update_pose(data))

        self._throttle = 0
        self._target_pitch = 0
        self._target_roll = 0
        self._target_heading = self._current_heading

    def _update_velocity(self, data):
        self._z_velocity = data['linear_velocity'][2]

    def _update_pose(self, data):
        self._altitude = data['z']
        self._current_heading = data['yaw']
        self._current_pitch = data['pitch']
        self._current_roll = data['roll']

    def loop(self):
        prop_speed = self._throttle * PROP_MAX_SPEED
        thrust = (prop_speed * PROP_PITCH - self._z_velocity) * MASS_PER_REV * prop_speed
        thrust = min(MAX_THRUST, max(thrust, 0))

        self._owner.set_thrust(thrust * NUMBER_OF_PROPS)

        self._owner.motion.publish({
            'roll': self._target_roll,
            'pitch': self._target_pitch,
            'yaw': self._target_heading,
            'thrust': 0.01
            })

    def set_throttle(self, throttle):
        if throttle > 1:
            throttle = 1
        elif throttle < 0:
            throttle = 0
        self._throttle = throttle

    def set_target_heading(self, heading):
        self._target_heading = heading

    def set_target_pitch(self, pitch):
        self._target_pitch = pitch

    def set_target_roll(self, roll):
        self._target_roll = roll

    def get_throttle(self):
        return self._throttle

    def get_target_heading(self):
        return self._target_heading

    def get_target_pitch(self):
        return self._target_pitch

    def get_target_roll(self):
        return self._target_roll

    def get_current_altitude(self):
        return self._altitude
