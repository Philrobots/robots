"""Factory used to create and control the instantiation of path-finding algorithms"""

from scripts.src.pathfinding.a_star import AStar
from scripts.src.pathfinding.pathfinding_algorithms import PathfindingAlgorithms


class PathfindingAlgorithmFactory:
    """Factory used to create and control the instantiation of path-finding algorithms"""
    @staticmethod
    def create(algorithm):
        """Create and controls the instantiation of path-finding algorithms"""
        if algorithm is PathfindingAlgorithms.A_STAR:
            return AStar()
        raise Exception("Chosen pathfinding algorithm has not yet been implemented.")
