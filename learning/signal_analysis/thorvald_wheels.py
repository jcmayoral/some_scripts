from thorvald_base.msg import BaseState
from sklearn.svm import SVC, OneClassSVM
from htf_velodyne_32e.msg import Velodyne_Gps
from fusion_msgs.msg import sensorFusionMsg
import numpy as np

import rospy

class SVMObserver:
    def __init__(self):
        rospy.init_node('svm_imu_test')
        self.is_training = True

        rospy.Subscriber('/base_state', BaseState, self.base_state_CB, queue_size=1)

        self.pub = list()
        self.clf = list()

        for i in range(4):
            self.clf.append(OneClassSVM(nu=0.3, kernel="poly", gamma=0.1))
            self.pub.append(rospy.Publisher('/observer_'+ str(i), sensorFusionMsg,queue_size=1))

        rospy.loginfo("Training period starting")
        rospy.Timer(rospy.Duration(20), self.timer_cb,oneshot=True)
        rospy.spin()

    def timer_cb(self, event):
        rospy.loginfo("Training period has ended")
        self.is_training = False

    def base_state_CB(self,msg):

        for i in range(4):
            X = np.array([[msg.prop_speed[i], msg.prop_pos[i], msg.steer_pos[i]]])
            fb_msg = sensorFusionMsg()
            fb_msg.sensor_id.data = "wheel_" + str(i)

            if self.is_training:
                self.clf[i].fit(X)
            else:
                detected_class = self.clf[i].predict(X)

                if detected_class < 0:
                    fb_msg.msg = sensorFusionMsg.ERROR
                    rospy.logwarn("Error " + str(i))

            self.pub[i].publish(fb_msg)


if __name__ == '__main__':
    SVMObserver()
