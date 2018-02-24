from morse.builder.creator import ActuatorCreator

class PlanarThrust(ActuatorCreator):
    _classpath = "sim.actuators.planarThrust.PlanarThrust"
    _blendname = "thrust"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

