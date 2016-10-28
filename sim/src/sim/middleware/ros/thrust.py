from std_msgs.msg import Float64
from morse.middleware.ros import ROSSubscriber

class ThrustReader(ROSSubscriber):
    """ Subscribe to a throttle topic and set thrust actuator data. """
    ros_class = Float64

    def update(self, message):
        self.data['throttle'] = message.data
