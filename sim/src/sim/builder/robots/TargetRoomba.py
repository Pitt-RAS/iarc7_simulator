from sim.builder.robots import Roomba
from sim.builder.sensors import TopTouchSensor

class TargetRoomba(Roomba):
    """
    A template robot model for TargetRoomba, with a motion controller and a pose sensor.
    """
    def __init__(self, name=None, debug=False):

        # TargetRoomba.blend is located in the data/robots directory
        Roomba.__init__(self, '', name, debug)
        self.properties(classpath = "sim.robots.TargetRoomba.TargetRoomba")

        ###################################
        # Sensors
        ###################################

        self.top_sensor = TopTouchSensor('TopTouchSensor')
        self.top_sensor.translate(0, 0, Roomba.ROOMBA_HEIGHT + 0.07)
        self.top_sensor.add_stream(
                'ros',
                'sim.middleware.ros.bump_sensor.BumpSensorPublisher',
                topic='/sim/{}/top_touch'.format(name))
        self.append(self.top_sensor)
