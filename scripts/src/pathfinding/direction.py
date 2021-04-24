"""Enum that specifies the directions the robot can move in its graph"""

from enum import Enum, auto


class Direction(Enum):
    """Enum that specifies the directions the robot can move in its graph"""
    RIGHT = auto()
    TOP_RIGHT = auto()
    UP = auto()
    TOP_LEFT = auto()
    LEFT = auto()
    DOWN_LEFT = auto()
    DOWN = auto()
    DOWN_RIGHT = auto()
