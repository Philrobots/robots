import rospy
from std_msgs.msg import String
from handlers.handler import Handler


class WaitForRobotReadyStateHandler(Handler):
    def __init__(self):
        self.rate = rospy.Rate(1)

    def initialize(self):
        self.sub = rospy.Subscriber("robot", String, self.handle_ready)
        self.is_finished = False

    def handle(self, handled_data):
        self.initialize()

        while not self.is_finished:
            rospy.logerr("waiting for robot ready")
            self.rate.sleep()

        return handled_data

    def handle_ready(self, _):
        self.is_finished = True

    def unregister(self):
        self.sub.unregister()
