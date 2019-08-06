# First import the library

from common.ros_subscriber import PointCloudSubscriber
from common.ros_publisher import PointCloudPublisher

#from common.copy_pointnet_eval import call
from common.pointnet2_class import ROSPointNet2

import numpy as np
import cv2
import rospy
import pcl
import sensor_msgs.point_cloud2
import ros_numpy
import signal
import copy

rospy.init_node("PointCloud2_test")

num_point = 512
custom_subscriber = PointCloudSubscriber("/velodyne_points/filtered")
publisher = PointCloudPublisher("/pointnet2/output")
ros_pointnet2 = ROSPointNet2(num_point=num_point)


def keyboardInterruptHandler(signal, frame):
    global ros_pointnet2
    print("KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(signal))
    rospy.logwarn("AAAPIHPUOG")
    ros_pointnet2.stop_call()
    exit(1)


signal.signal(signal.SIGINT, keyboardInterruptHandler)


try:
    while True:
        msg = custom_subscriber.get_current_msg()
        pc = np.zeros((msg.height * msg.width, 3))

        pc_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

        p = pcl.PointCloud()
        p.from_array(np.array(pc_array,dtype=np.float32))

        tree = p.make_kdtree()
        ec = p.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.15)
        ec.set_MinClusterSize(num_point)
        #ec.set_MaxClusterSize(num_point*1.5)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()

        if len(cluster_indices) == 0:
            rospy.logwarn("Clusters not found")

        for j, indices in enumerate(cluster_indices):
            # cloudsize = indices
            #print('indices = ' + str(len(indices)))
            # cloudsize = len(indices)
            points = np.zeros((len(indices), 3), dtype=np.float32)
            # points = np.zeros((cloudsize, 3), dtype=np.float32)

            # for indice in range(len(indices)):
            for i, indice in enumerate(indices):
                # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
                # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
                points[i][0] = p[indice][0]
                points[i][1] = p[indice][1]
                points[i][2] = p[indice][2]

            publisher.custom_publish(points, frame_id = "velodyne")

            prepared_points = copy.deepcopy(points)
            for i in range(1024/num_point -2):
                prepared_points = np.vstack((prepared_points, points))
            print prepared_points.shape
            ros_pointnet2.call(prepared_points)
            rospy.sleep(4.5)



finally:
    #LOG_FOUT.close()
    rospy.loginfo("finishing")
