import json

import rospy
from handlers.handler import Handler
from std_msgs.msg import String


class MoveRobotToResistanceStationHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.robot_pose = None

    def initialize(self):
        # self.move_robot_handler = MoveRobotHandler()
        self.sub = rospy.Subscriber("robot", String, self.callback, queue_size=1)
        self.rate = rospy.Rate(0.2)
        self.initialized = True

    def callback(self, data):
        robot_dict = json.loads(data.data)
        self.robot_pose = robot_dict["robot"]
        self.sub.unregister()

    def handle(self, handled_data=None):
        if not self.initialized:
            self.initialize()
        while self.robot_pose is None:
            pass

        x_dist = abs(handled_data["RESISTANCE_STATION"][0] - self.robot_pose[0])/handled_data["convertion_to_cm"]
        # rospy.logerr("X_DIST" + str(x_dist))
        handled_data["resistance_x_dist"] = x_dist
        handled_data["movement_vectors_string_pub"].publish(json.dumps((x_dist, 0, 1)))
        self.rate.sleep()


        y_dist = abs(handled_data["RESISTANCE_STATION"][1] - self.robot_pose[1])/handled_data["convertion_to_cm"]
        handled_data["resistance_y_dist"] = y_dist
        # rospy.logerr("Y_DIST" + str(y_dist))
        # rospy.logerr(handled_data)
        handled_data["movement_vectors_string_pub"].publish(json.dumps((y_dist, 0, 2)))
        self.rate.sleep()


        return handled_data

    def unregister(self):
        self.sub.unregister()
