#!/usr/bin/env python
import sys
import pyjulius
import Queue
import rospy
from std_msgs.msg import String

class JuliusROSCLient():
    def __init__(self):
        # Initialize and try to connect
        self.client = pyjulius.Client('localhost', 10500)
        #ROS
        rospy.init_node("ros_julius",disable_signals=True)
        self.publisher = rospy.Publisher("speech_recognizing", String, queue_size=1)

    def initialize(self):
        try:
            self.client.connect()
            print ("Connected")
        except pyjulius.ConnectionError:
            print 'Start julius as module first!'
            sys.exit(1)
        self.client.start()
        print ("Starting")

    def run(self):
        try:
                try:
                   result = self.client.results.get(False)
                except Queue.Empty:
                    return

                #Publishing
                if isinstance(result, pyjulius.models.Sentence):
                    msg = String()
                    print ("Sentence Detected")
                    msg.data = str(result)
                    self.publisher.publish(msg)

        except KeyboardInterrupt:
            print 'Exiting...'
            self.client.stop()  # send the stop signal
            self.client.join()  # wait for the thread to die
            self.client.disconnect()  # disconnect from julius

# Start listening to the server
client = JuliusROSCLient()
client.initialize()

while not rospy.is_shutdown():
    client.run()
