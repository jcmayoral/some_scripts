# First import the library

from common.pc_visualizer import showpoints
import pyrealsense2 as rs
import numpy as np
import cv2

import matplotlib.pyplot as plt



# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()

try:
    for aaa in range(5):
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


        for x in range(depth_image.shape[0]):
            for y in range(depth_image.shape[1]):
                depth_value = depth_raw[x*depth_image.shape[1] + y]
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], depth_value)
                pc[x*depth_image.shape[1] + y] = depth_point


        idx = np.random.randint(depth_image.shape[0] * depth_image.shape[1], size=500)
        hist, bin_edges = np.histogram(pc[idx,:], density=False)

        plt.hist(hist, bins="auto")
        plt.show(False)

        showpoints(pc[idx,:])


finally:
    pipeline.stop()
