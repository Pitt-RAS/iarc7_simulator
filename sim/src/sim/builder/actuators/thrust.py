from morse.builder.creator import ActuatorCreator

class Thrust(ActuatorCreator):
    _classpath = "sim.actuators.thrust.Thrust"
    _blendname = "thrust"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

