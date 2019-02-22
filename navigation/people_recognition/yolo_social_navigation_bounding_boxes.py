import rospy
from people_msgs.msg import Person, People
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from tf import TransformListener
import random

class PeopleSocialLayer:
    def __init__(self):
        rospy.init_node("people_layer_hack")
        self.fb_people = People()
        self.fb_people.header.frame_id = 'map'
        #self.tf_listener = TransformListener()
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.sub_cb, queue_size=10)
        self.pub = rospy.Publisher("/people", People, queue_size=10)
        self.found_objects = dict()
        rospy.spin()

    def sub_cb(self, msg):
        self.fb_people.header.stamp = rospy.Time.now()

        for box in msg.bounding_boxes:
            if box.Class!="truck":# and box.probability > 0.2:
                if box.Class in self.found_objects:
                    self.fb_people.people[self.found_objects[box.Class]].position.x = random.random()*10
                    self.fb_people.people[self.found_objects[box.Class]].position.y = random.random()*10
                else:
                    self.found_objects[box.Class]=len(self.fb_people.people)
                    fb_person = Person()
                    print box.Class, " Found"
                    p1 = PoseStamped()
                    p1.header.frame_id = "base_link"
                    p1.pose.position.x = random.random()*10
                    p1.pose.position.y = random.random()*10
                    p1.pose.orientation.w = 1.0    # Neutral orientation
                    #p_in_base = self.tf_listener.transformPose("map", p1)

                    #fb_person.position.x = p_in_base.pose.position.x
                    #fb_person.position.y = p_in_base.pose.position.y
                    fb_person.position.x = p1.pose.position.x
                    fb_person.position.y = p1.pose.position.y
                    fb_person.name = str(box.Class)
                    self.fb_people.people.append(fb_person)
        self.pub.publish(self.fb_people)


if __name__ == '__main__':
    PeopleSocialLayer()
