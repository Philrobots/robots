from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler
from utils import create_pose


class MoveRobotToCommandPanelHandler(Handler):
    def __init__(self):
        self.initialized = False

    def initialize(self):
        self.move_robot_handler = MoveRobotHandler()
        self.initialized = True

    def handle(self, handled_data=None):
        if not self.initialized:
            self.initialize()

        handled_data["goal"] = create_pose(handled_data["COMMAND_PANEL"])
        handled_data["path_following_mode_pub"].publish("CENTER")

        handled_data = self.move_robot_handler.handle(handled_data)

        return handled_data

    def unregister(self):
        self.move_robot_handler.unregister()
