# First import the library

from common.ros_publisher import PointCloudPublisher
from common.copy_pointnet_eval import call
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
import pcl
import matplotlib.pyplot as plt


rospy.init_node("PointCloud2_test")

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()

publisher = PointCloudPublisher("/pointnet2/input")

try:
    for aaa in range(1):
        # Create a pipeline object. This object configures the streaming camera and owns it's handle
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        if not depth:
            exit()

        depth_data = depth.as_frame().get_data()
        rgb_data = depth.as_frame().get_data()

        depth_image = np.asanyarray(depth_data)
        rgb_image = np.asanyarray(rgb_data)
        depth_intrin = depth.profile.as_video_stream_profile().intrinsics

        pc = np.zeros((depth_image.shape[0]*depth_image.shape[1], 3))
        depth_raw = depth_image.flatten()

        mask = np.ones((depth_image.shape[0]*depth_image.shape[1], 3))


        for x in range(depth_image.shape[0]):
            for y in range(depth_image.shape[1]):
                depth_value = depth_raw[x*depth_image.shape[1] + y]
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], depth_value)
                pc[x*depth_image.shape[1] + y] = depth_point
                mask[x*depth_image.shape[1] + y] = 0

        print "count zeros", np.count_nonzero(mask)

        #idx = np.random.randint(depth_image.shape[0] * depth_image.shape[1], size=500)
        call(pc)
        p = pcl.PointCloud()
        p.from_array(np.array(pc,dtype=np.float32))
        sor = p.make_voxel_grid_filter()
        sor.set_leaf_size(0.1, 0.1, 0.1)
        cloud_filtered = sor.filter()

        publisher.custom_publish(pc, frame_id = "camera")

finally:
    #LOG_FOUT.close()
    pipeline.stop()
