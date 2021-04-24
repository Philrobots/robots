import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler
from mapping.resistance import Resistance
from mapping.resistance_mapper import ResistanceMapper


class ReadResistanceHandler(Handler):
    def initialize(self):
        self.sub = rospy.Subscriber('resistance', String, self.read_resistance)
        self.rate = rospy.Rate(0.5)
        self.is_finished = False

    def handle(self, handled_data=None):
        self.initialize()

        while not self.is_finished:
            handled_data["movement_vectors_string_pub"].publish(json.dumps((0, 0, 6)))
            self.rate.sleep()

        handled_data['resistance'] = self.resistance
        resistance_object = Resistance(self.resistance)
        rounded_resistance, colors = resistance_object.get_resistance_and_colors()
        colors = colors + [ResistanceMapper().find_exponent_color(resistance_object)]
        rospy.logerr([rounded_resistance, colors])
        handled_data["puck_colors"] = colors
        handled_data["puck_colors_pub"].publish(json.dumps(colors))

        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((handled_data["resistance_y_dist"], 0, 3)))
        self.rate.sleep()
        handled_data["movement_vectors_string_pub"].publish(json.dumps((handled_data["resistance_x_dist"], 0, 0)))
        self.rate.sleep()

        return handled_data

    def read_resistance(self, data):
        resistance = json.loads(data.data)
        self.resistance = resistance
        self.is_finished = resistance != -1

    def unregister(self):
        self.sub.unregister()
