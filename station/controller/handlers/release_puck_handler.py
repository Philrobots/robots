import json
import math
import rospy
from std_msgs.msg import String

from handlers.handler import Handler


class ReleasePuckHandler(Handler):
    def __init__(self):
        self.initialized = False
        self.rate = rospy.Rate(0.5)
        self.forwards_rate = rospy.Rate(1)
        self.DROP = 8
        self.UP = 9
        self.LOWER = 10
        self.BRAKES = 11
        self.position_tuple = None
        self.robot_angle = None

    def initialize(self):
        self.sub = rospy.Subscriber("robot", String, self.callback, queue_size=1)
        self.initialized = True
    def callback(self, data):
        robot_dict = json.loads(data.data)
        self.position_tuple = robot_dict["prehenseur"]
        self.robot_angle = robot_dict["angle"]


    def handle(self, handled_data=None):
        self.initialize()
        while self.position_tuple is None:
            pass
        while self.distance(self.position_tuple, (handled_data["goal"].pose.position.x, handled_data["goal"].pose.position.y)) > 20:
            sauce = self.distance(self.position_tuple, (handled_data["goal"].pose.position.x, handled_data["goal"].pose.position.y))

            vector_angle = self.get_angle_between_two_points(handled_data["goal"].pose.position.x, handled_data["goal"].pose.position.y, *self.position_tuple)
            correction_angle = self.get_angle_correction(self.robot_angle, vector_angle)

            if abs(correction_angle) > math.radians(5):
                correction_angle = 0

            handled_data["movement_vectors_string_pub"].publish(json.dumps((1, math.degrees(correction_angle), 0)))
            self.forwards_rate.sleep()
            handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0, 11)))

        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.DROP)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0 ,self.UP)))
        self.rate.sleep()

        handled_data["movement_vectors_string_pub"].publish(json.dumps((10, 0 , 1)))
        self.rate.sleep()

        return handled_data

    def get_angle_correction(self, robot_angle, vector_angle):
        """
        Changes the vector orientation from the absolute value from the top camera
        to the angle the robot will need to use to align itself with the vector.
        (For one vector)
        """
        if vector_angle < 0:
            vector_angle = 2 * math.pi + vector_angle
        if robot_angle < 0:
            robot_angle = 2 * math.pi + robot_angle

        angle_correction = vector_angle - robot_angle

        if angle_correction > math.pi:
            angle_correction -= 2 * math.pi
        elif angle_correction < -math.pi:
            angle_correction += 2 * math.pi
        return angle_correction

    def get_angle_between_two_points(self, x1, y1, x2, y2):
        angle = -math.atan2(y2 - y1, x2 - x1)

        if angle == -0:
            angle = 0
        elif angle == -math.pi:
            angle = math.pi
        return angle

    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))

    def unregister(self):
        self.sub.unregister()
