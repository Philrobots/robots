import json

import rospy
from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler
from std_msgs.msg import String
from utils import create_pose


class MoveRobotToNextPuckHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.goal = None
        self.current_puck_color= None
        self.goal_tuple = None

    def initialize(self):
        self.pub = rospy.Publisher('movement_vectors_string', String, queue_size=1)
        self.sub = rospy.Subscriber('pucks', String, self.pucks_callback, queue_size=1)
        self.move_robot_handler = MoveRobotHandler()
        self.initialized = True

    def pucks_callback(self, data):
        self.handled_data["calculate_pucks_pub"].publish(True)
        pucks_dict = json.loads(data.data)
        self.goal_tuple = pucks_dict[self.current_puck_color][0]["center_position"]
        self.sub.unregister()

    def handle(self, handled_data=None):
        #handled_data["calculate_pucks_pub"].publish(True)
        self.handled_data = handled_data
        if not self.initialized:
            self.initialize()

        self.current_puck_color = handled_data['puck_colors'].pop(0)

        while self.goal_tuple is None or self.current_puck_color is None:
            pass

        handled_data["goal"] = create_pose(self.goal_tuple)

        handled_data["path_following_mode_pub"].publish("PUCK")
        handled_data = self.move_robot_handler.handle(handled_data)


        return handled_data

    def unregister(self):
        self.pub.unregister()
        self.sub.unregister()
        self.move_robot_handler.unregister()
