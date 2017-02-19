from sim.builder.robots import Roomba

class Obstacle(Roomba):
    """
    A template robot model for Obstacle, with a motion controller and a pose sensor.
    """
    def __init__(self, name=None, debug=False):

        # Obstacle.blend is located in the data/robots directory
        Roomba.__init__(self, 'sim/robots/Obstacle.blend', name, debug)
        self.properties(classpath = "sim.robots.Obstacle.Obstacle")
