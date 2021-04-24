"""Node class that represents each square from the matrix drawn on the image"""

from scripts.src.pathfinding.tile_role import TileRole


class Node:
    """Node class that represents each square from the matrix drawn on the image"""
    def __init__(self, matrix_center, pixel_coordinates_center, width, height):
        self.matrix_center = matrix_center
        self.pixel_coordinates_center = pixel_coordinates_center
        self.width = width
        self.height = height
        self.neighbors = []
        self.role = TileRole.EMPTY
        self.held_by = set()
        self.uuid = None
