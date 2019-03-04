#!/usr/bin/python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from numpy import fabs,sqrt, floor
from numpy.linalg import norm
global publisher


def topic_cb(msg):
    gen = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z","intensity" ))
    transformed_image = Image()
    transformed_image.header.frame_id = "velodyne"
    x_resolution = 0.1
    y_resolution = 0.1
    ray_number = 600#int(3 / x_resolution)#220
    print ray_number
    size =  msg.height * msg.width
    transformed_image.height = ray_number#fabs(floor(sqrt(size)))#
    transformed_image.width = ray_number#fabs(floor(sqrt(size)))#
    size =  int(transformed_image.height * transformed_image.width)

    transformed_image.encoding = "mono8"
    data = [0] * transformed_image.height * transformed_image.width

    max_distance = 25 #max distance of lidar
    min_distance = -3.5#-0.05 #for some reason is negative
    scale_factor = 110
    max_value = 255

    ignored_points =0
    miscalculation = 0

    for i in range(size-1, -1, -1):
        try:
            point = gen.next()
            x = point[0]
            y = point[1]
            cell_x = round(x/x_resolution)
            cell_y = round(y/y_resolution)
            raw_value = gen.next()[2]
        except:
            continue
            print "NO"

        #print raw_value
        value = min(fabs(raw_value / (max_distance- min_distance))*max_value, max_value)
        #value = 100
        #if raw_value > -1:
        #    value = max_value
        #value = 255

        cell_x = (transformed_image.height/2) + cell_x

        if cell_x > transformed_image.height:
            cell_x = cell_x - transformed_image.height
            miscalculation = miscalculation+1
            continue

        cell_y = (transformed_image.width/2) + cell_y

        if cell_y > transformed_image.width:
            cell_y = cell_y - transformed_image.width
            miscalculation = miscalculation+1
            continue

        index =  int(fabs(cell_x + transformed_image.width * cell_y))

        data[min(index, size -1)] = min(fabs(value), max_value)

        #data[i] = min(fabs(value), max_value)
    print "total ignored points ", ignored_points
    print "total miscalculation points ", miscalculation
    transformed_image.data = data
    publisher.publish(transformed_image)

rospy.init_node("gazebo_3d_lidar_to_image")

publisher = rospy.Publisher("/scan_velodyne_hack/image_raw", Image, queue_size=1)
rospy.Subscriber("/velodyne_points", PointCloud2, topic_cb, queue_size=1)
rospy.spin()
