import json

import rospy
from handlers.handler import Handler
from std_msgs.msg import String


class MoveRobotHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.is_finished = False
        self.rate = rospy.Rate(0.3)
        self.vector = None

    def initialize(self):
        self.sub = rospy.Subscriber('movement_vectors_string', String, self.is_vector_at_destination)
        self.initialized = True

    def handle(self, handled_data=None):

        if not self.initialized:
            self.initialize()

        while self.vector is None:
            handled_data["goal_pub"].publish(handled_data['goal'])
            self.rate.sleep()

        while not self.is_finished:
            pass

        return handled_data

    def is_vector_at_destination(self, vector_json):
        rospy.logerr(vector_json.data)
        if vector_json.data == "FINISHED":
            self.vector = "FINISHED"
        else:
            self.vector = json.loads(str(vector_json.data))

        self.is_finished = self.vector == "FINISHED"

    def unregister(self):
        self.sub.unregister()
