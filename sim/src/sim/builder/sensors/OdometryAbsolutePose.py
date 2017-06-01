from morse.builder import Odometry

class OdometryAbsolutePose(Odometry):
    _classpath = "sim.sensors.OdometryAbsolutePose.OdometryAbsolutePose"

    def __init__(self, name=None):
        Odometry.__init__(self, name)
