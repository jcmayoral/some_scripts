#!/usr/bin/python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from numpy import fabs
global publisher


def topic_cb(msg):
    gen = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z","intensity" ))
    transformed_image = Image()
    transformed_image.header.frame_id = "velodyne"
    transformed_image.height = msg.width/128
    transformed_image.width = 128
    transformed_image.encoding = "mono8"
    data = [0] * transformed_image.height * transformed_image.width

    max_distance = 10 #max distance of lidar
    min_distance = -0.05 #for some reason is negative
    scale_factor = 1

    for i in range(transformed_image.height * transformed_image.width-1, -1, -1):
        value = gen.next()[2] - min_distance
        data[i] = fabs(scale_factor * value*255/(max_distance - min_distance))

    transformed_image.data = data
    publisher.publish(transformed_image)

rospy.init_node("gazebo_3d_lidar_to_image")

publisher = rospy.Publisher("/scan_velodyne_hack", Image, queue_size=10)
rospy.Subscriber("scan_velodyne", PointCloud2, topic_cb, queue_size=10)
rospy.spin()
