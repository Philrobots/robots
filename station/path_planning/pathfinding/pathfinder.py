"""Module used to find and draw a path from a point to another using a map representation"""


class Pathfinder:
    """Class used to find and draw a path from a point to another using a map representation"""
    def __init__(self, _map, pathfinding_algorithm):
        self._map = _map
        self.pathfinding_algorithm = pathfinding_algorithm
        self.path = []

    def find_square_matrix_path(self):
        """Finds the path from the starting node to the end node"""
        start = self._map.get_start_node()
        end = self._map.get_end_node()
        self.path = self.pathfinding_algorithm.find_path(start, end)

    def find_path_to_empty_space(self):
        start = self._map.get_start_node()
        return self.pathfinding_algorithm.find_path_to_empty_space(start)