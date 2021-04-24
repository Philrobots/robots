from enum import Enum


class ObstacleRepresentation(Enum):
    RADIUS = 0
    DIAGONAL = 1
    WIDTH_HEIGHT = 2
    RADIUS_WIDTH_HEIGHT = 3
    SQUARE = 4
