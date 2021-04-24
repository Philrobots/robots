import math

from enum import Enum


class RobotDirections(Enum):
    FORWARD = 0
    FORWARD_RIGHT = -math.pi/4
    RIGHT = -math.pi/2
    BACKWARDS_RIGHT = -3*math.pi/4
    BACKWARDS = math.pi
    BACKWARDS_LEFT = 3*math.pi/4
    LEFT = math.pi/2
    FORWARD_LEFT = math.pi/4

    def get_mode_mapping(self):
        return {
            self.FORWARD: 0,
            self.FORWARD_RIGHT: 12,
            self.RIGHT: 2,
            self.BACKWARDS_RIGHT: 14,
            self.BACKWARDS: 1,
            self.BACKWARDS_LEFT: 15,
            self.LEFT: 3,
            self.FORWARD_LEFT: 13
        }