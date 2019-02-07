#!/usr/bin/python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from numpy import fabs
global publisher


def unsigned(n):
    return int(n) & 0xFFFFFFFF

def topic_cb(msg):
    gen = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z", ))
    transformed_image = Image()
    transformed_image.header.frame_id = "velodyne"
    transformed_image.height = msg.width/128
    transformed_image.width = 128#msg.width#/128
    transformed_image.encoding = "mono8"
    data = [0] * transformed_image.height * transformed_image.width

    scale_factor = 5

    for i in range(transformed_image.height * transformed_image.width-1, 1, -1):
        value = abs(gen.next()[2])
        data[i] = min(255,value*255/scale_factor) #int(p[2])])
    transformed_image.data = data
    publisher.publish(transformed_image)

rospy.init_node("gazebo_3d_lidar_to_image")

publisher = rospy.Publisher("/scan_velodyne_hack", Image, queue_size=10)
rospy.Subscriber("scan_velodyne", PointCloud2, topic_cb, queue_size=10)
rospy.spin()
