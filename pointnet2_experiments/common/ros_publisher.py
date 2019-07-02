import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

class PointCloudPublisher(rospy.Publisher):
    def __init__(self, topic_name, msg_class = PointCloud2):
        rospy.Publisher.__init__(self, topic_name, msg_class, queue_size=1)

    def custom_publish(self, points, stamp=None, frame_id=None):

        msg = PointCloud2()
        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id

        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)

        msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False

        msg.point_step = 12
        #addPointField(msg_pointcloud, format_str.c_str(), 1, sensor_msgs::PointField::UINT32, msg_pointcloud.point_step);
        msg.row_step = points.shape[0] * points.shape[1] #*msg.point_step

        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()
        self.publish(msg)

        """


        print "A"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera"

        print "b"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          #PointField('rgb', 12, PointField.UINT32, 1),
          # PointField('rgba', 12, PointField.UINT32, 1),
          ]
        print "c"
        pc2 = point_cloud2.create_cloud(header, fields, points)
        self.publish(pc2)

        """
