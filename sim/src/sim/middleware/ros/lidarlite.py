from std_msgs.msg import Float64
from morse.middleware.ros import ROSPublisher

class LidarLitePublisher(ROSPublisher):
    """ Publish data from a LidarLite sensor. """
    ros_class = Float64

    def default(self, ci='unused'):
        self.publish(self.data['range'])
