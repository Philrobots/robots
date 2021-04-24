import json

import rospy

from handlers.handler import Handler
from std_msgs.msg import String
import math


class GripPuckHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.robot_pose = None
        self.rate = rospy.Rate(0.5)
        self.GRAB = 7
        self.DROP = 8
        self.RAISE = 9
        self.LOWER = 10
        self.cm_to_pixel = 6.882391855

    def initialize(self):
        self.sub = rospy.Subscriber("robot", String, self.callback, queue_size=1)
        self.initialized = True

    def handle(self, handled_data=None):
        if not self.initialized:
            self.initialize()
        while self.robot_pose is None:
            pass

        #length = distance(*handled_data['goal'], *self.robot_pose) + 15

        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.DROP)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((15, 0 , 0)))
        #handled_data["movement_vectors_string_pub"].publish(json.dumps((length/self.cm_to_pixel, 0 , 0)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.GRAB)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((20, 0 , 1))) #TODO: ici il recule de 5 de plus qu'il avance
        #handled_data["movement_vectors_string_pub"].publish(json.dumps((length/self.cm_to_pixel, 0 , 1)))
        self.rate.sleep()
        return handled_data

    def callback(self, data):
        robot_dict = json.loads(data.data)
        self.robot_pose = robot_dict["robot"]
        self.sub.unregister()

    def unregister(self):
        self.sub.unregister()


def distance(x1, y1, x2, y2):
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
