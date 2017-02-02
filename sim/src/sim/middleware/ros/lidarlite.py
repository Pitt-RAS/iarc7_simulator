from iarc7_msgs.msg import Float64Stamped
from morse.middleware.ros import ROSPublisher

class LidarLitePublisher(ROSPublisher):
    """ Publish data from a LidarLite sensor. """
    ros_class = Float64Stamped

    def default(self, ci='unused'):
        msg = Float64Stamped()
        msg.header = self.get_ros_header()

        try:
            msg.header.frame_id = self.kwargs['frame']
        except KeyError:
            msg.header.frame_id = ''

        msg.data = self.data['range']
        self.publish(msg)
