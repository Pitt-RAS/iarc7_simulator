from iarc7_msgs.msg import BoolStamped
from morse.middleware.ros import ROSPublisher

class BumpSensorPublisher(ROSPublisher):
    """ Publish data from a bump sensor. """
    ros_class = BoolStamped

    def default(self, ci='unused'):
        msg = BoolStamped()
        msg.header = self.get_ros_header()
        msg.data = self.data['positive']
        self.publish(msg)
