from morse.builder.creator import SensorCreator

class LidarLite(SensorCreator):
    _classpath = "sim.sensors.LidarLite.LidarLite"

    def __init__(self, name = None):
        SensorCreator.__init__(self, name)
