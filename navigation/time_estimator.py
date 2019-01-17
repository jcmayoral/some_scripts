import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Vector3
import numpy as np

class TimeEstimator:
    def __init__(self):
        rospy.init_node("time_estimator")
        self.mean_primitive_error = 0
        self.samples = 0
        self.is_training = True
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.PathCB, queue_size=1)
        rospy.Subscriber("/motion_finished", Empty, self.MotionCompleteCB, queue_size=1)

        self.start_time = rospy.Time.now()
        self.estimated_time = 0
        self.lenght = 1
        self.K =0
        self.A = list()
        self.y = list()
        self.coefficients = np.zeros(6)
        rospy.spin()

    def train_data(self):
        if len(self.y) > 2:
            self.coefficients = np.linalg.lstsq(self.A, self.y)[0]

    def PathCB(self,msg):
        self.lenght = len(msg.poses)
        simple_approximation = 0.1*len(msg.poses)
        rospy.logwarn("Simple Time Estimation %f " % simple_approximation)

        self.start_time = rospy.Time.now()

        p0 = msg.poses[0]
        self.K = 0

        dx = list()
        dy = list()
        ddx = list()
        ddy = list()

        for p in msg.poses[1:]:
            dx.append(p.pose.position.x - p0.pose.position.x)
            dy.append(p.pose.position.y - p0.pose.position.y)
            p0 = p

        dx0 = dx[0]
        dy0 = dy[0]

        for x,y in zip(dx[1:],dy[1:]):
            ddx.append(x - dx0)
            ddy.append(y - dy0)
            dx0 = x
            dy0 = y

        dx = np.sum(dx,axis=0)
        dy = np.sum(dy,axis=0)
        ddx = np.sum(ddx, axis=0)
        ddy = np.sum(ddy,axis=0)

        curvature = (ddy * dx - ddx * dy) / (np.power(dx, 2) + np.power(dy, 2))
        self.features = [dx,dy,ddx,ddy,curvature, self.lenght]

        if self.samples >0:
            statistic_estimation = self.estimated_time + self.lenght* self.mean_primitive_error/self.samples
            rospy.logwarn("Statistical Estimation %f" , statistic_estimation)

        lst_estimated_time = np.sum(self.coefficients * np.array([dx, dy, ddx, ddy,curvature, self.lenght]))
        rospy.logwarn("Complete Linearization Estimation %f " % lst_estimated_time)


    def MotionCompleteCB(self,msg):
        measured_time = (rospy.Time.now() - self.start_time).to_sec()

        self.samples = self.samples+1
        self.mean_primitive_error += (measured_time - self.estimated_time)/self.lenght

        rospy.logerr("Measured Time %f",  measured_time)

        self.A.append(self.features)
        self.y.append(measured_time)

        self.train_data()


TimeEstimator()
