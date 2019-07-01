import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class PointCloudSubscriber(rospy.Subscriber):
    def __init__(self, topic_name, msg_class = PointCloud2, onesshot = True):
        self._current_msg = None
        if onesshot:
            self.one_shot(topic_name, msg_class)
            return
        rospy.Publisher.__init__(self, topic_name, msg_class, self.cb, queue_size=1)

    def one_shot(self,topic, msg_class):
        msg = rospy.wait_for_message(topic, msg_class)
        self._current_msg = msg
        print "NOT NONE"

    def cb(self, msg):
        self._current_msg = msg

    def get_current_msg(self):
        return self._current_msg
