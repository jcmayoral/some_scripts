#!/usr/bin/python
import numpy as np
from sklearn.svm import SVC, OneClassSVM
from sensor_msgs.msg import NavSatFix
from fusion_msgs.msg import sensorFusionMsg

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import rospy

class SVMObserver:
    def __init__(self):
        rospy.init_node('svm_imu_test')
        self.is_training = True
        self.clf = OneClassSVM(nu=0.3, kernel="poly", gamma=0.1)
        self.pub = rospy.Publisher('/collisions_0', sensorFusionMsg, queue_size=1)
        rospy.loginfo("Training period starting")

        self._f, self._ax = plt.subplots(3, 1)#, subplot_kw=dict(polar=True))
        self.color = ["r", "b", "g", "k"]
        self.sub = rospy.Subscriber('/phone1/android/fix', NavSatFix, self.gps_CB, queue_size=1)
        #rospy.Timer(rospy.Duration(10), self.timer_cb,oneshot=True)
        plt.show()
        rospy.spin()

    def timer_cb(self, event):
        rospy.loginfo("Training period has ended")
        self.is_training = False

    def gps_CB(self,msg):
        X = np.array([[msg.latitude, msg.longitude, msg.altitude]])
        fb_msg = sensorFusionMsg()
        fb_msg.sensor_id.data = "GPS"
        """
        if self.is_training:
            self.clf.fit(X)

        else:
            detected_class = self.clf.predict(X)

            if detected_class < 0:
                fb_msg.msg = sensorFusionMsg.ERROR
                rospy.logwarn('Event Detected')
        self.pub.publish(fb_msg)
        """

        self._ax[0].scatter(X.reshape(3,-1)[0,-1],X.reshape(3,-1)[1,-1], color =self.color[-1])
        self._ax[1].scatter(X.reshape(3,-1)[0,-1],X.reshape(3,-1)[2,-1], color =self.color[-2])
        self._ax[2].scatter(X.reshape(3,-1)[1,-1],X.reshape(3,-1)[2,-1], color =self.color[-3])

        plt.draw()
        #plt.show(False)
        rospy.sleep(0.1)


if __name__ == '__main__':
    SVMObserver()
