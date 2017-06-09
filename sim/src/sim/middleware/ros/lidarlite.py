from sensor_msgs.msg import Range
from morse.middleware.ros import ROSPublisher

class LidarLitePublisher(ROSPublisher):
    """ Publish data from a LidarLite sensor. """
    ros_class = Range

    def default(self, ci='unused'):
        msg = Range()
        msg.header = self.get_ros_header()

        try:
            msg.header.frame_id = self.kwargs['frame']
        except KeyError:
            msg.header.frame_id = ''

        msg.radiation_type = Range.INFRARED
        msg.min_range = self.component_instance.bge_object['min_range']
        msg.max_range = self.component_instance.bge_object['max_range']
        msg.range = self.data['range']
        self.publish(msg)
