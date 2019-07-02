# First import the library
from common.ros_publisher import PointCloudPublisher
#from common.copy_pointnet_eval import call, stop_call
from common.pointnet2_class import ROSPointNet2
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
import pcl
import matplotlib.pyplot as plt
import signal
import random

from scipy.linalg import svd


rospy.init_node("PointCloud2_test")

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
#pipeline.start()
config = rs.config()
pipeline.start()


#depth_sensor = profile.get_device().first_depth_sensor()
#depth_scale = depth_sensor.get_depth_scale()
#print

num_point = 300
original_publisher = PointCloudPublisher("/pointnet2/input")
publisher = PointCloudPublisher("/pointnet2/output")
ros_pointnet2 = ROSPointNet2(num_point=num_point)

def keyboardInterruptHandler(signal, frame):
    global ros_pointnet2
    global pipeline
    print("KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(signal))
    rospy.logwarn("AAAPIHPUOG")
    pipeline.stop()
    ros_pointnet2.stop_call()
    exit(1)

signal.signal(signal.SIGINT, keyboardInterruptHandler)

while True:
        # Create a pipeline object. This object configures the streaming camera and owns it's handle
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        rs_pc = rs.pointcloud()
        points = rs_pc.calculate(depth_frame)
        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        p = pcl.PointCloud()
        p.from_array(np.array(verts,dtype=np.float32))

        original_publisher.custom_publish(verts, frame_id = "camera")


        #print "original", p.size
        #Downsampling
        sor = p.make_voxel_grid_filter()
        sor.set_leaf_size(0.075, 0.075, 0.075)
        cloud_filtered = sor.filter()
        #print "after Downsampling", cloud_filtered.size


        #Passtrough filter
        passthrough = cloud_filtered.make_passthrough_filter()
        #passthrough = p.make_passthrough_filter()
        passthrough.set_filter_field_name("z")
        passthrough.set_filter_limits(0.0, 2.0)
        cloud_filtered = passthrough.filter()
        #print "after passthrough", cloud_filtered.size

        #cluster extraction
        tree = cloud_filtered.make_kdtree()
        ec = cloud_filtered.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.2)
        ec.set_MinClusterSize(num_point)
        ec.set_MaxClusterSize(num_point*1.5)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()

        """
        #passthrough X,Y
        passthrough = cloud_filtered.make_passthrough_filter()
        passthrough.set_filter_field_name("x")
        passthrough.set_filter_limits(-1.0, 1.0)
        cloud_filtered = passthrough.filter()

        passthrough = cloud_filtered.make_passthrough_filter()
        passthrough.set_filter_field_name("y")
        passthrough.set_filter_limits(-1.0, 1.0)cloud_filtered = passthrough.filter()
        """


        """
        verts = cloud_filtered.to_array()
        #RANDOM
        verts = np.asarray(random.sample(verts, 100))
        #U, S, V = svd(verts,full_matrices=True)
        #verts = np.matrix(U[:, :3]) * np.diag(S[:3]) * np.matrix(V[:3, :])

        ros_pointnet2.call(verts)
        publisher.custom_publish(verts, frame_id = "camera")
        rospy.sleep(2)
        """

        #CLUSTERING

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
                points[i][0] = cloud_filtered[indice][0]
                points[i][1] = cloud_filtered[indice][1]
                points[i][2] = cloud_filtered[indice][2]
            ros_pointnet2.call(points)
            publisher.custom_publish(points, frame_id = "camera")
            rospy.sleep(2)
            #cloud_cluster.from_array(points)


rospy.logerr("ENDING")
