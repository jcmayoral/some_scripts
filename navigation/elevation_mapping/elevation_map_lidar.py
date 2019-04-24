#!/usr/bin/python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image, Imu
from htf_velodyne_32e.msg import Velodyne_Gps
from numpy import fabs,sqrt, floor
from math import cos, sin
from numpy.linalg import norm
from tf.transformations import euler_from_quaternion

class Lidar2ElevationMap:
    def __init__(self):
        rospy.init_node("lidar_to_image")
        self.transformed_image = Image()
        self.transformed_image.header.frame_id = "velodyne"
        self.x_resolution = 0.1
        self.y_resolution = 0.1

        self.ray_number = 600
        self.transformed_image.height = self.ray_number
        self.transformed_image.width = self.ray_number
        self.transformed_image.encoding = "mono8"

        self.min_distance = 150
        self.max_value = 255
        self.robot_pose = [0,0]
        self.init_robot_pose = None
        self.last_angle = 0.0

        self.size =  int(self.ray_number*self.ray_number)
        self.transformed_image.data = [0] * self.size

        self.publisher = rospy.Publisher("/scan_velodyne_hack/image_raw", Image, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.topic_cb, queue_size=1)
        #rospy.Subscriber("/Velodyne_Lidar/gps", Velodyne_Gps, self.pose_cb, queue_size=1)
        rospy.Subscriber("/VectorNav_IMU/imu", Imu, self.imu_cb, queue_size=1)

        rospy.loginfo("Node initialized")
        rospy.spin()

    def imu_cb(self, msg):
        angle = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2]
        self.robot_pose[0] += msg.linear_acceleration.x  * cos(angle-self.last_angle)
        self.robot_pose[1] += msg.linear_acceleration.y  * sin(angle-self.last_angle)
        print self.robot_pose
        self.last_angle = angle


    def pose_cb(self, msg):
        if self.init_robot_pose is None:
            self.init_robot_pose = [msg.latitude, msg.longitude]
            return
        self.robot_pose[0] += msg.latitude - self.init_robot_pose[0]
        self.robot_pose[1] += msg.longitude - self.init_robot_pose[1]
        print self.robot_pose


    def topic_cb(self,msg):
        gen = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z","intensity" ))

        for i in range(self.size-1, -1, -1):
            try:
                point = gen.next()
                x = point[0]
                y = point[1]
                z = point[2]
                intensity = point[3]
                cell_x = round(x/self.x_resolution)
                cell_y = round(y/self.y_resolution)
                feature = point[3]/point[2]
            except:
                continue

            if norm([x,y])> self.min_distance:
                continue

            cell_x = int(fabs(self.robot_pose[0]/self.x_resolution + cell_x))

            if cell_x > self.ray_number:
                continue

            cell_y = int(fabs(self.robot_pose[1]/self.y_resolution + cell_y))

            if cell_y > self.ray_number:
                continue

            index =  int(fabs(cell_x + self.ray_number * cell_y))

            feature = 1+self.transformed_image.data[min(index, self.size -1)]
            self.transformed_image.data[min(index, self.size -1)] = min(fabs(feature), self.max_value)

        self.transformed_image.header.stamp = rospy.Time.now()
        self.publisher.publish(self.transformed_image)



if __name__ == '__main__':
    Lidar2ElevationMap()
