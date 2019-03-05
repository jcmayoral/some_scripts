#!/usr/bin/python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from numpy import fabs,sqrt, floor
from numpy.linalg import norm

class Lidar2Image:
    def __init__(self):
        rospy.init_node("lidar_to_image")
        self.transformed_image = Image()
        self.transformed_image.header.frame_id = "velodyne"
        self.x_resolution = 0.05
        self.y_resolution = 0.05

        self.ray_number = 600
        self.transformed_image.height = self.ray_number
        self.transformed_image.width = self.ray_number
        self.transformed_image.encoding = "mono8"

        self.max_distance = 20
        self.max_value = 255

        self.size =  int(self.ray_number*self.ray_number)
        self.publisher = rospy.Publisher("/scan_velodyne_hack/image_raw", Image, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.topic_cb, queue_size=1)
        rospy.loginfo("Node initialized")
        rospy.spin()


    def topic_cb(self,msg):
        gen = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z","intensity" ))


        data = [0] * self.size

        for i in range(self.size-1, -1, -1):
            try:
                point = gen.next()
                x = point[0]
                y = point[1]
                cell_x = round(x/self.x_resolution)
                cell_y = round(y/self.y_resolution)
                raw_value = point[3]/point[2]
            except:
                continue
                print "NO"

            if norm([x,y])> self.max_distance:
                continue

            cell_x = (self.ray_number/2) + cell_x

            if cell_x > self.ray_number:
                continue

            cell_y = (self.ray_number/2) + cell_y

            if cell_y > self.ray_number:
                continue

            index =  int(fabs(cell_x + self.ray_number * cell_y))

            data[min(index, self.size -1)] = min(fabs(raw_value), self.max_value)

        self.transformed_image.data = data
        self.publisher.publish(self.transformed_image)



if __name__ == '__main__':
    Lidar2Image()
