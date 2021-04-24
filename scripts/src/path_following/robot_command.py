import math

from enum import Enum


class RobotCommand(Enum):
    FORWARD = 0
    FORWARD_RIGHT = -math.pi/4
    RIGHT = -math.pi/2
    BACKWARDS_RIGHT = -3*math.pi/4
    BACKWARDS = math.pi
    BACKWARDS_LEFT = 3*math.pi/4
    LEFT = math.pi/2
    FORWARD_LEFT = math.pi/4
    #GRAB
    #RELEASE
