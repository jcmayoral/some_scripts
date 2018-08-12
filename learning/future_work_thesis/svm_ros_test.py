import numpy as np
from sklearn.svm import SVC, OneClassSVM

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool,String

class SVMObserver:
    def __init__(self):
        rospy.init_node('svm_imu_test')
        rospy.Subscriber('/android/imu', Imu, self.imuCB)
        rospy.Subscriber('mode', Bool, self.modeCB)

        self.is_training = True
        self.clf = OneClassSVM(nu=0.5, kernel="poly", gamma=0.1)
        rospy.spin()

    def modeCB(self,msg):
        self.is_training = msg.data

    def imuCB(self,msg):

        X = np.array([[msg.linear_acceleration.x, msg.linear_acceleration.y,
                       msg.angular_velocity.z]])

        if self.is_training:
            self.clf.fit(X)

        else:
            if self.clf.predict(X) > 0:
                rospy.logwarn('COLLISION')
            else:
                rospy.loginfo('OK')


if __name__ == '__main__':
    SVMObserver()
