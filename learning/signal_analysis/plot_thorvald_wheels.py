from thorvald_base.msg import BaseState
from sklearn.decomposition import PCA
from htf_velodyne_32e.msg import Velodyne_Gps
from fusion_msgs.msg import sensorFusionMsg
from std_msgs.msg import Empty
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


import rospy

class PCAObserver:
    def __init__(self):
        rospy.init_node('svm_imu_test')
        self.is_training = True

        self.pca = PCA(n_components=3)

        #rospy.loginfo("Training period starting")
        #rospy.Timer(rospy.Duration(3), self.timer_cb,oneshot=True)
        self.X4 = np.array([])

        #self._f = plt.figure()
        #self._ax = plt.axes()
        self._f, self._ax = plt.subplots(3, 1)#, subplot_kw=dict(polar=True))

        self.color = ["r", "b", "g", "k"]
        self.flag = True
        #rospy.Subscriber('/color_select', Empty, self.color_cb, queue_size=1)
        rospy.Subscriber('/base_state', BaseState, self.base_state_CB, queue_size=1)
        plt.show()

        rospy.spin()

    def color_cb(self, msg):
        if self.flag:
            self.color = "b"
        else:
            self.color = "r"
        self. flag = not self.flag

    def timer_cb(self, event):
        rospy.loginfo("Training period has ended")
        self.is_training = False

    def base_state_CB(self,msg):
        self.X4 = np.array([])

        for i in range(4):
            X = np.array([msg.prop_speed[i], msg.prop_pos[i], msg.steer_pos[i]])
            self.X4 = np.append(self.X4,X)


        if self.is_training:
            pass
            #return

        self.pca.fit(self.X4.reshape(3,-1))

        #print self.pca.transform(X.reshape(3,-1))
        #print self.pca.explained_variance_

        #for x, y in zip(self.pca.explained_variance_, self.pca.components_):
        #    for i in y:

        #for x,y in zip (self.X4.reshape(3,-1)[0], self.X4.reshape(3,-1)[1]):
        #    self._ax.scatter(self.X4.reshape(3,-1)[0],self.X4.reshape(3,-1)[1], color =self.color)
        for i in range(1,5):
            print -i
            print self.X4.reshape(3,-1).shape
            self._ax[0].scatter(self.X4.reshape(3,-1)[0,-i],self.X4.reshape(3,-1)[1,-i], color =self.color[-i])
            self._ax[1].scatter(self.X4.reshape(3,-1)[0,-i],self.X4.reshape(3,-1)[2,-i], color =self.color[-i])
            self._ax[2].scatter(self.X4.reshape(3,-1)[1,-i],self.X4.reshape(3,-1)[2,-i], color =self.color[-i])

        plt.draw()
        #plt.show(False)
        rospy.sleep(0.1)
        #detected_class = self.clf[i].predict(X)

        #if detected_class < 0:
        #    fb_msg.msg = sensorFusionMsg.ERROR
        #    rospy.logwarn("Error " + str(i))


if __name__ == '__main__':
    PCAObserver()
