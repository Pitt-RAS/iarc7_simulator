from morse.builder.creator import ComponentCreator, SensorCreator
import os

class ContactSwitch(SensorCreator):
    _classpath = "sim.sensors.ContactSwitch.ContactSwitch"
    _blendname = os.path.realpath(os.path.join(
        os.path.dirname(__file__),
        "../../../../data/sim/sensors/ContactSwitch.blend"))

    def __init__(self, name = None):
        SensorCreator.__init__(self, name, ComponentCreator.USE_BLEND)

