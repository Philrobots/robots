import pytest

from scripts.src.pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from scripts.src.pathfinding.a_star import AStar
from scripts.src.pathfinding.pathfinding_algorithms import PathfindingAlgorithms


class TestPathfindingAlgorithmFactory:
    """Test path-finding algorithm factory"""
    @classmethod
    def setup_class(cls):
        cls.A_STAR_ALGORITHM = PathfindingAlgorithms.A_STAR
        cls.A_STAR_CLASS = AStar
        cls.NOT_IMPLEMENTED_ALGORITHM_NAME = "whatever"

    def setup_method(self):
        self.pathfinding_algorithm_factory = PathfindingAlgorithmFactory()

    def test_when_create_with_a_star_algorithm_then_return_a_star_object(self):
        algorithm = self.pathfinding_algorithm_factory.create(
            self.A_STAR_ALGORITHM)

        assert isinstance(algorithm, self.A_STAR_CLASS)

    def test_when_create_with_not_implemented_algorithm_then_raise_exception(self):
        with pytest.raises(Exception):
            self.pathfinding_algorithm_factory.create(self.NOT_IMPLEMENTED_ALGORITHM_NAME)
