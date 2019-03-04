#!/usr/bin/python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from numpy import fabs,sqrt
global publisher


def topic_cb(msg):
    gen = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z","intensity" ))
    transformed_image = Image()
    transformed_image.header.frame_id = "velodyne"
    ray_number = 240
    size =  msg.height * msg.width
    transformed_image.height = ray_number#fabs(round(sqrt(size)))
    transformed_image.width = msg.width/ray_number#fabs(round(sqrt(size)))
    size =  int(transformed_image.height * transformed_image.width)

    transformed_image.encoding = "mono8"
    data = [0] * transformed_image.height * transformed_image.width

    max_distance = 2 #max distance of lidar
    min_distance = 0.2#-0.05 #for some reason is negative
    scale_factor = 110
    max_value = 255
    resolution = 0.2

    for i in range(size-1, -1, -1):
        try:
            point = gen.next()
            x = point[0]
            cell_x = int(x/resolution)
            y = point[1]
            cell_y = int(y/resolution)
            raw_value = gen.next()[2]
        except:
            continue
            print "NO"

        value = (raw_value + min_distance)/max_distance* max_value
        #value = 255

        cell_x = (transformed_image.width/2) + cell_x
        cell_y = (transformed_image.height/2) + cell_y

        index =  int(fabs(cell_x + transformed_image.width * cell_y))

        data[min(index, size -1)] = min(fabs(value), max_value)
        #data[i] = min(fabs(value), max_value)

    transformed_image.data = data
    publisher.publish(transformed_image)

rospy.init_node("gazebo_3d_lidar_to_image")

publisher = rospy.Publisher("/scan_velodyne_hack", Image, queue_size=10)
rospy.Subscriber("/velodyne_points", PointCloud2, topic_cb, queue_size=10)
rospy.spin()
