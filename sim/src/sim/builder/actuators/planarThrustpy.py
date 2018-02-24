from morse.builder.creator import ActuatorCreator

class Planarthrustpy(ActuatorCreator):
    _classpath = "sim.actuators.planarThrustpy.Planarthrustpy"
    _blendname = "planarThrustpy"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

