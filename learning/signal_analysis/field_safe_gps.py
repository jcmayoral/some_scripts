#!/usr/bin/python
import numpy as np
from sklearn.svm import SVC, OneClassSVM
from htf_velodyne_32e.msg import Velodyne_Gps
from fusion_msgs.msg import sensorFusionMsg

import rospy

class SVMObserver:
    def __init__(self):
        rospy.init_node('svm_imu_test')
        self.is_training = True
        self.clf = OneClassSVM(nu=0.3, kernel="poly", gamma=0.1)
        self.sub = rospy.Subscriber('/Velodyne_Lidar/gps', Velodyne_Gps, self.imuCB, queue_size=1)
        self.pub = rospy.Publisher('/collisions_0', sensorFusionMsg, queue_size=1)
        rospy.loginfo("Training period starting")
        rospy.Timer(rospy.Duration(20), self.timer_cb,oneshot=True)
        rospy.spin()

    def timer_cb(self, event):
        rospy.loginfo("Training period has ended")
        self.is_training = False

    def imuCB(self,msg):
        X = np.array([[msg.latitude, msg.longitude, msg.speed]])
        fb_msg = sensorFusionMsg()
        fb_msg.sensor_id.data = "GPS"

        if self.is_training:
            self.clf.fit(X)

        else:
            detected_class = self.clf.predict(X)

            if detected_class < 0:
                fb_msg.msg = sensorFusionMsg.ERROR
                rospy.logwarn('Event Detected')
        self.pub.publish(fb_msg)


if __name__ == '__main__':
    SVMObserver()
