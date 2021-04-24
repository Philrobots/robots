import pytest

from scripts.src.pathfinding.node import Node
from scripts.src.pathfinding.a_star import AStar
from scripts.src.pathfinding.path_not_found_exception import PathNotFoundException


class TestAStar:
    @classmethod
    def setup_class(cls):
        cls.A_WIDTH = 2
        cls.A_HEIGHT = 3
        cls.SOME_MATRIX_COORDINATES = (4, 5)
        cls.SOME_OTHER_MATRIX_COORDINATES = (6, 7)
        cls.SOME_PIXEL_COORDINATES = (8, 9)
        cls.SOME_OTHER_PIXEL_COORDINATES = (12, 12)
        cls.SOME_ANGLE = "40"
        cls.A_MATRIX_POSITION = (11, 15)
        cls.A_PIXEL_POSITION = (1000, 1000)
        cls.A_WIDTH = 12
        cls.A_HEIGHT = 9
        cls.END_NODE = Node(
            cls.A_MATRIX_POSITION,
            cls.A_PIXEL_POSITION,
            cls.A_WIDTH,
            cls.A_HEIGHT)
        cls.START_NODE = Node(
            cls.SOME_MATRIX_COORDINATES,
            cls.SOME_PIXEL_COORDINATES,
            cls.A_WIDTH,
            cls.A_HEIGHT
        )
        cls.OTHER_NODE = Node(
            cls.SOME_OTHER_MATRIX_COORDINATES,
            cls.SOME_OTHER_PIXEL_COORDINATES,
            cls.A_WIDTH,
            cls.A_HEIGHT
        )

    def setup_method(self):
        self.a_star = AStar()
        self.END_NODE = Node(
            self.A_MATRIX_POSITION,
            self.A_PIXEL_POSITION,
            self.A_WIDTH,
            self.A_HEIGHT)
        self.START_NODE = Node(
            self.SOME_MATRIX_COORDINATES,
            self.SOME_PIXEL_COORDINATES,
            self.A_WIDTH,
            self.A_HEIGHT
        )
        self.OTHER_NODE = Node(
            self.SOME_OTHER_MATRIX_COORDINATES,
            self.SOME_OTHER_PIXEL_COORDINATES,
            self.A_WIDTH,
            self.A_HEIGHT
        )

    def test_given_graph_where_goal_is_not_reachable_when_find_path_then_raise_path_not_found(self):
        start_node = self.given_graph_where_goal_is_not_reachable()

        with pytest.raises(PathNotFoundException):
            self.a_star.find_path(start_node, self.END_NODE)

    def test_given_graph_where_goal_is_reachable_when_find_path_then_path_is_valid(self):
        start_node = self.given_graph_where_goal_is_reachable()

        path = self.a_star.find_path(start_node, self.OTHER_NODE)

        assert path

    def given_graph_where_goal_is_reachable(self):
        self.START_NODE.neighbors.append((self.OTHER_NODE, None))
        return self.START_NODE

    def given_graph_where_goal_is_not_reachable(self):
        self.START_NODE.neighbors.append((self.OTHER_NODE, None))
        return self.START_NODE
