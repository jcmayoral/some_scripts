import numpy as np
from sklearn.svm import SVC, OneClassSVM

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool,String
from people_msgs.msg import Person, People
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from tf import TransformListener

class SVMObserver:
    def __init__(self):
        rospy.init_node("imu_anomaly_detection_layer_hack")
        self.tf_listener = TransformListener()
        rospy.Subscriber('/android/imu', Imu, self.imuCB, queue_size=1)
        rospy.Subscriber('mode', Bool, self.modeCB)
        self.pub = rospy.Publisher("/people", People, queue_size=10)
        self.step = 0.4
        self.counter = 1
        self.fb_people = People()
        self.fb_people.header.frame_id = 'map'
        self.is_training = True
        self.clf = OneClassSVM(nu=0.5, kernel="poly", gamma=0.5)
        rospy.spin()

    def publish_pose(self):
        self.fb_people.header.stamp = rospy.Time.now()
        fb_person = Person()

        p1 = PoseStamped()
        p1.header.frame_id = "base_link"
        p1.pose.position.x = 1
        p1.pose.orientation.w = 1.0    # Neutral orientation
        p_in_base = self.tf_listener.transformPose("map", p1)

        fb_person.position.x = p_in_base.pose.position.x
        fb_person.position.y = p_in_base.pose.position.y

        fb_person.name = str(fb_person.position.x + fb_person.position.y)
        rospy.loginfo("Object Found")
        self.fb_people.people.append(fb_person)
        self.pub.publish(self.fb_people)

    def modeCB(self,msg):
        self.is_training = msg.data

    def imuCB(self,msg):

        X = np.array([[msg.linear_acceleration.x, msg.linear_acceleration.y,
                       msg.angular_velocity.z]])

        if self.is_training:
            self.clf.fit(X)

        else:
            if self.clf.predict(X) > 0:
                rospy.logwarn('Situation Found')
                self.publish_pose()

if __name__ == '__main__':
    SVMObserver()
