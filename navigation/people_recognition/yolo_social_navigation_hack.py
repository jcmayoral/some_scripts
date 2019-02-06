import rospy
from people_msgs.msg import Person, People
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from tf import TransformListener

class PeopleSocialLayer:
    def __init__(self):
        rospy.init_node("people_layer_hack")
        self.tf_listener = TransformListener()
        self.sub = rospy.Subscriber("/darknet_ros/found_object", Int8, self.sub_cb, queue_size=10)
        self.pub = rospy.Publisher("/people", People, queue_size=10)
        self.step = 0.4
        self.counter = 1
        self.fb_people = People()
        self.fb_people.header.frame_id = 'map'
        rospy.spin()

    def sub_cb(self, msg):
        self.fb_people.header.stamp = rospy.Time.now()
        fb_person = Person()

        if msg.data>2:
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


if __name__ == '__main__':
    PeopleSocialLayer()
