# First import the library

from common.ros_subscriber import PointCloudSubscriber
from common.copy_pointnet_eval import call
import numpy as np
import cv2
import rospy
import pcl
import sensor_msgs.point_cloud2
import ros_numpy


rospy.init_node("PointCloud2_test")

custom_subscriber = PointCloudSubscriber("/velodyne_points/filtered")

try:
    for aaa in range(1):

        msg = custom_subscriber.get_current_msg()
        pc = np.zeros((msg.height * msg.width, 3))

        rospy.sleep(3)
        pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        print "WORKS", pc.shape
        #idx = np.random.randint(dmpth_image.shape[0] * depth_image.shape[1], size=500)
        call(pc)
finally:
    #LOG_FOUT.close()
    rospy.loginfo("finishing")
