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

rospy.init_node("PointCloud2_test")

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
#pipeline.start()
config = rs.config()
pipeline.start()


#depth_sensor = profile.get_device().first_depth_sensor()
#depth_scale = depth_sensor.get_depth_scale()
#print

publisher = PointCloudPublisher("/pointnet2/input")
ros_pointnet2 = ROSPointNet2()

def keyboardInterruptHandler(signal, frame):
    global ros_pointnet2
    global pipeline
    print("KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(signal))
    rospy.logwarn("AAAPIHPUOG")
    ros_pointnet2.stop_call()
    pipeline.stop()
    exit(0)

signal.signal(signal.SIGINT, keyboardInterruptHandler)

while True:
        # Create a pipeline object. This object configures the streaming camera and owns it's handle
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        rs_pc = rs.pointcloud()
        print rs_pc
        points = rs_pc.calculate(depth_frame)
        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        p = pcl.PointCloud()
        p.from_array(np.array(verts,dtype=np.float32))
        #Downsampling
        #sor = p.make_voxel_grid_filter()
        #sor.set_leaf_size(0.07, 0.07, 0.07)
        #cloud_filtered = sor.filter()

        #Passtrough filter
        #passthrough = cloud_filtered.make_passthrough_filter()
        passthrough = p.make_passthrough_filter()
        passthrough.set_filter_field_name("z")
        passthrough.set_filter_limits(0.0, 1.5)
        cloud_filtered = passthrough.filter()

        """
        passthrough = cloud_filtered.make_passthrough_filter()
        passthrough.set_filter_field_name("x")
        passthrough.set_filter_limits(-1.0, 1.0)
        cloud_filtered = passthrough.filter()

        passthrough = cloud_filtered.make_passthrough_filter()
        passthrough.set_filter_field_name("y")
        passthrough.set_filter_limits(-1.0, 1.0)
        cloud_filtered = passthrough.filter()
        """
        verts = cloud_filtered.to_array()

        verts = np.asarray(random.sample(verts, 500))

        ros_pointnet2.call(verts)
        publisher.custom_publish(verts, frame_id = "camera")
        rospy.sleep(2)

rospy.logerr("ENDING")
