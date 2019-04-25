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
        self.x_resolution = 0.5
        self.y_resolution = 0.5

        self.pixels_number = 200
        self.transformed_image.height = self.pixels_number
        self.transformed_image.width = self.pixels_number
        self.transformed_image.encoding = "mono8"

        self.max_value = 255

        self.size =  int(self.pixels_number*self.pixels_number)
        self.transformed_image.data = [0] * self.size

        self.publisher = rospy.Publisher("/scan_velodyne_hack/image_raw", Image, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.topic_cb, queue_size=1)
        rospy.loginfo("Node initialized")
        rospy.spin()


    def topic_cb(self,msg):
        self.transformed_image.data = [0] * self.size

        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z","intensity" ))

        for i in range(self.size-1, -1, -1):
            print "@"
            try:
                point = gen.next()
                x = point[0]
                y = point[1]
                z = point[2]
                intensity = point[3]
                cell_x = round(x*self.x_resolution)
                cell_y = round(y*self.y_resolution)
                feature = point[3]/point[2]
            except:
                print "error"
                continue

            cell_x = (self.pixels_number/2) + cell_x
            print cell_x

            #if cell_x > self.ray_number:
            #    continue

            cell_y = (self.pixels_number/2) + cell_y
            print cell_y
            #if cell_y > self.ray_number:
            #    continue

            index =  int(fabs(cell_x + self.pixels_number * cell_y))
            if self.size - index < 0:
                print "error"
                return

            self.transformed_image.data[index] = min(fabs(feature), self.max_value)

        self.transformed_image.header.stamp = rospy.Time.now()
        self.publisher.publish(self.transformed_image)



if __name__ == '__main__':
    Lidar2Image()
