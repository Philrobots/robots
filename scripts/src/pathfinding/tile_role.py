"""Enum that specifies the different roles the nodes can have in the graph"""

from enum import Enum, auto


class TileRole(Enum):
    """Enum that specifies the different roles the nodes can have in the graph"""
    START = auto()
    END = auto()
    OBSTACLE = auto()
    CUSHION = auto()
    PUCK = auto()
    EMPTY = auto()
