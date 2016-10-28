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

    def loop(self):
        self._owner.thrust.publish({'throttle': self._throttle})

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
