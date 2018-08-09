import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Vector3
import numpy as np

class TimeEstimator:
    def __init__(self):
        rospy.init_node("time_estimator")
        self.accumulator = 0
        self.samples = 0
        self.is_training = True
        rospy.Subscriber("/global_planner/plan", Path, self.PathCB, queue_size=1)
        rospy.Subscriber("/local_planner/isGoalReached", Empty, self.MotionCompleteCB, queue_size=1)
        rospy.Subscriber("/time_estimator/training", Bool, self.trainingCB, queue_size=1)
        self.feedback_pub = rospy.Publisher("time_estimator/fb", Vector3)
        self.start_time = rospy.Time.now()
        self.estimated_time = 0
        self.lenght = 1
        self.weight = 0
        self.K =0
        rospy.spin()

    def trainingCB(self,msg):
        self.is_training = msg.data
        if not self.is_training:
            print "OFF SET per pixel is ", self.accumulator/self.samples

    def PathCB(self,msg):
        self.lenght = len(msg.poses)
        self.estimated_time = 0.1*len(msg.poses)
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

        self.K = (ddy * dx - ddx * dy) / (np.power(dx, 2) + np.power(dy, 2))
        #self.K/=self.lenght

        #print "Curvature " , self.K 

        if self.samples >0:# not self.is_training:
            self.estimated_time = self.estimated_time - self.lenght * (self.accumulator/self.samples + self.K)
            #self.estimated_time = self.estimated_time - (self.accumulator/self.samples)*self.lenght + self.K * self.lenght / self.weight
        print "Estimated TIME ", self.estimated_time

    def MotionCompleteCB(self,msg):
        measured_time = (rospy.Time.now() - self.start_time).to_sec()

        if self.is_training:
            self.samples = self.samples+1
            self.accumulator += (self.estimated_time - measured_time)/self.lenght
            self.weight += (self.estimated_time - measured_time)

        print "MEASURED TIME", measured_time
        print "Percentage", (measured_time- self.estimated_time)/measured_time

        fb_msgs = Vector3()
        fb_msgs.x = np.fabs(measured_time - self.estimated_time)/measured_time
        fb_msgs.y = self.lenght
        fb_msgs.z = self.K
        self.feedback_pub.publish(fb_msgs)
        #print "ERROR per primitive ", (self.estimated_time - measured_time)/self.lenght

TimeEstimator()
