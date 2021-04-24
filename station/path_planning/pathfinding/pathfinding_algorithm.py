"""Interface detailing the explicit contract of a path-finding algorithm"""


class PathfindingAlgorithm:
    """Interface detailing the explicit contract of a path-finding algorithm"""
    def find_path(self, start, end):
        """Finds a path from the start node to the end node."""
        raise NotImplementedError()
