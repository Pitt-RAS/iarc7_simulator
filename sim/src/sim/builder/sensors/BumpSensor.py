from morse.builder.creator import ComponentCreator, SensorCreator
from morse.builder.creator import SensorCreator
import os

class BumpSensor(SensorCreator):
    _classpath = "sim.sensors.BumpSensor.BumpSensor"
    _blendname = os.path.realpath(os.path.join(
        os.path.dirname(__file__),
        "../../../../data/sim/sensors/BumpSensor.blend"))

    def __init__(self, name = None):
        SensorCreator.__init__(self, name, ComponentCreator.USE_BLEND)

