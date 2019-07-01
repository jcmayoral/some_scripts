# First import the library

from common.ros_publisher import PointCloudPublisher
from common.copy_pointnet_eval import call, stop_call
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
import pcl
import matplotlib.pyplot as plt


rospy.init_node("PointCloud2_test")

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
#pipeline.start()
config = rs.config()
pipeline.start()
rs_pc = rs.pointcloud()


#depth_sensor = profile.get_device().first_depth_sensor()
#depth_scale = depth_sensor.get_depth_scale()
#print

publisher = PointCloudPublisher("/pointnet2/input")

try:
    while 1:
        # Create a pipeline object. This object configures the streaming camera and owns it's handle
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        points = rs_pc.calculate(depth_frame)
        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

        p = pcl.PointCloud()
        p.from_array(np.array(verts,dtype=np.float32))
        sor = p.make_voxel_grid_filter()
        sor.set_leaf_size(0.1, 0.1, 0.1)
        cloud_filtered = sor.filter()

        pc = cloud_filtered.to_array()
        #print "after filtering ", pc.shape

        call(pc)
        #call(verts)
        publisher.custom_publish(pc, frame_id = "camera")
        rospy.sleep(3)
        print "f"

finally:
    stop_call()
    print "g"
    pipeline.stop()
    print "h"
