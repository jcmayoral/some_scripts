import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool

class TimeEstimator:
    def __init__(self):
        rospy.init_node("time_estimator")
        self.samples = 0
        self.is_training = True
        rospy.Subscriber("/global_planner/plan", Path, self.PathCB, queue_size=1)
        rospy.Subscriber("/local_planner/isGoalReached", Empty, self.MotionCompleteCB, queue_size=1)
        rospy.Subscriber("/time_estimator/training", Bool, self.trainingCB, queue_size=1)
        self.start_time = rospy.Time.now()
        self.estimated_time = 0
        self.lenght = 1
        self.weight = 0
        self.K =0
        rospy.spin()

    def trainingCB(self,msg):
        self.is_training = msg.data

    def PathCB(self,msg):
        self.lenght = len(msg.poses)
        self.estimated_time = 0.1*len(msg.poses)
        self.start_time = rospy.Time.now()
        #print "Estimated TIME ", self.estimated_time

        p0 = msg.poses[0]
        self.K = 0

        for p in msg.poses[1:]:
            self.K = self.K + (p.pose.position.x -p0.pose.position.x) 
            self.K = self.K + (p.pose.position.y -p0.pose.position.y) 

        self.K = self.K/len(msg.poses)
        #print "Curvature " , self.K 
        if self.samples >0:# not self.is_training:
            self.estimated_time = self.estimated_time - (self.weight/self.samples)*self.lenght + self.K / self.weight
        print "Estimated TIME ", self.estimated_time

    def MotionCompleteCB(self,msg):
        measured_time = (rospy.Time.now() - self.start_time).to_sec()

        if self.is_training:
            self.samples = self.samples+1
            self.weight += (self.estimated_time - measured_time)/self.lenght

        print "MEASURED TIME", measured_time
        print "ERROR IN SECONDS", self.estimated_time - measured_time
        #print "ERROR per primitive ", (self.estimated_time - measured_time)/self.lenght

TimeEstimator()
