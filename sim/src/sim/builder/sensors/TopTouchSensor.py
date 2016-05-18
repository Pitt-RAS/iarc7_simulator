from morse.builder.creator import ComponentCreator, SensorCreator
from morse.builder import bpymorse
import os

class TopTouchSensor(SensorCreator):
    _classpath = "sim.sensors.TopTouchSensor.TopTouchSensor"
    _blendname = os.path.realpath(os.path.join(
        os.path.dirname(__file__),
        "../../../../data/sim/sensors/TopTouchSensor.blend"))

    def __init__(self, name = None):
        SensorCreator.__init__(self, name, ComponentCreator.USE_BLEND)
