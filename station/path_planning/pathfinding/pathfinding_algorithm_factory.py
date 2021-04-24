"""Factory used to create and control the instantiation of path-finding algorithms"""

from pathfinding.breadth_first_search import BreadthFirstSearch
from pathfinding.a_star import AStar
from pathfinding.pathfinding_algorithms import PathfindingAlgorithms


class PathfindingAlgorithmFactory:
    """Factory used to create and control the instantiation of path-finding algorithms"""
    @staticmethod
    def create(algorithm):
        """Create and controls the instantiation of path-finding algorithms"""
        if algorithm is PathfindingAlgorithms.BREADTH_FIRST_SEARCH:
            return BreadthFirstSearch()
        elif algorithm is PathfindingAlgorithms.A_STAR:
            return AStar()
        raise Exception("Chosen pathfinding algorithm has not yet been implemented.")
