import uuid

from scripts.src.pathfinding.map import Map
from scripts.src.pathfinding.tile_role import TileRole


class TestMap:
    """Test Map class"""
    @classmethod
    def setup_class(cls):
        cls.AN_IMAGE_WIDTH = 300
        cls.AN_IMAGE_HEIGHT = 300
        cls.SOME_OBSTACLES = [
            (0, 0),
            (250, 250)
        ]
        cls.SOME_PUCKS = [
            (110, 23),
            (300, 300)
        ]
        cls.AN_ENDING_POSITION = (40, 60)
        cls.A_STARTING_POSITION = (213, 170)
        cls.A_NODE_SIZE = 25
        cls.A_SAFETY_CUSHION = 20
        cls.A_ROBOT_WIDTH = 100
        cls.AN_OBSTACLE_WIDTH = 40
        cls.A_PUCK_WIDTH = 25
        cls.A_NODE_POSITION = (5, 5)
        cls.A_PIXEL_POSITION = (55, 55)
        cls.A_DISTANCE = 2
        cls.AN_EMPTY_DISTANCE = 0
        cls.A_NEGATIVE_DISTANCE = -1
        cls.A_CUSHION_ROLE = TileRole.CUSHION
        cls.A_MATRIX_POSITION = (8, 6)

        cls.EXPECTED_NUMBER_OF_COLUMNS = 13
        cls.EXPECTED_NUMBER_OF_NODES_PER_ROW = 13
        cls.EXPECTED_NUMBER_OF_TOTAL_NODES = 169

        cls.EXPECTED_NUMBER_OF_NEIGHBORS = {2, 3, 4, 5, 6, 7, 8, 9}

        cls.EXPECTED_NEIGHBOR_DISTANCE = {1, 2}
        cls.EXPECTED_MATRIX_POSITION = (6, 8)

    def setup_method(self):
        self.expected_cushion_nodes_position = {
            (5, 3): 0,
            (4, 4): 0,
            (5, 4): 0,
            (6, 4): 0,
            (3, 5): 0,
            (4, 5): 0,
            (5, 5): 0,
            (6, 5): 0,
            (7, 5): 0,
            (4, 6): 0,
            (5, 6): 0,
            (6, 6): 0,
            (5, 7): 0
        }
        self.map = Map(
            self.AN_IMAGE_WIDTH,
            self.AN_IMAGE_HEIGHT,
            self.A_NODE_SIZE,
            self.A_SAFETY_CUSHION,
            self.A_ROBOT_WIDTH,
            self.AN_OBSTACLE_WIDTH,
            self.A_PUCK_WIDTH)

    def test_when_create_nodes_then_node_matrix_is_not_empty(self):
        self.map.create_nodes()

        assert self.map.node_matrix

    def test_when_create_nodes_then_number_of_nodes_in_each_row_is_as_expected(self):
        self.map.create_nodes()

        assert len(self.map.node_matrix[0]) == self.EXPECTED_NUMBER_OF_NODES_PER_ROW

    def test_when_create_nodes_then_number_of_columns_is_as_expected(self):
        self.map.create_nodes()

        assert len(self.map.node_matrix) == self.EXPECTED_NUMBER_OF_COLUMNS

    def test_when_create_nodes_then_number_of_nodes_created_is_as_expected(self):
        self.map.create_nodes()

        assert sum([len(row) for row in self.map.node_matrix]) == \
               self.EXPECTED_NUMBER_OF_TOTAL_NODES

    def test_when_connect_nodes_then_each_node_has_an_expected_number_of_neighbors(self):
        self.map.create_nodes()

        self.map.connect_nodes()

        for line in self.map.node_matrix:
            for node in line:
                assert len(node.neighbors) in self.EXPECTED_NUMBER_OF_NEIGHBORS

    def test_when_connect_nodes_then_each_neighbor_is_an_expected_distance_away(self):
        self.map.create_nodes()

        self.map.connect_nodes()

        for line in self.map.node_matrix:
            for node in line:
                first_node_x, first_node_y = node.matrix_center

                for neighbor, _ in node.neighbors:
                    second_node_x, second_node_y = neighbor.matrix_center
                    distance = abs(second_node_x-first_node_x) + abs(second_node_y-first_node_y)

                    assert distance in self.EXPECTED_NEIGHBOR_DISTANCE

    def test_when_render_map_then_node_matrix_is_not_empty(self):
        self.map.render_map()

        assert self.map.node_matrix

    def test_when_get_node_from_pixel_then_return_expected_node(self):
        self.map.create_nodes()

        node = self.map.get_node_from_pixel(self.A_STARTING_POSITION)

        assert node.matrix_center == self.EXPECTED_MATRIX_POSITION

    def test_when_get_node_from_matrix_coordinates_then_return_expected_node(self):
        self.map.create_nodes()

        node = self.map.get_node_from_matrix_coordinates(self.A_MATRIX_POSITION)

        assert node.matrix_center == self.EXPECTED_MATRIX_POSITION

    def test_when_add_top_wall_then_there_are_obstacles_at_the_top(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        self.map.table_walls_start_x = 0
        self.map.table_walls_end_x = self.AN_IMAGE_WIDTH
        self.map.table_walls_start_y = 0
        self.map.table_walls_end_y = self.AN_IMAGE_HEIGHT
        width = 1

        self.map.add_top_wall(width)

        assert all([node.role is TileRole.OBSTACLE for node in self.map.node_matrix[0]])

    def test_when_add_bottom_wall_then_there_are_obstacles_at_the_bottom(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        self.map.table_walls_start_x = 0
        self.map.table_walls_end_x = self.AN_IMAGE_WIDTH
        self.map.table_walls_start_y = 0
        self.map.table_walls_end_y = self.AN_IMAGE_HEIGHT
        width = 1

        self.map.add_bottom_wall(width)

        assert all([node.role is TileRole.OBSTACLE for node in self.map.node_matrix[-1]])

    def test_when_add_left_wall_then_there_are_obstacles_to_the_left(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        self.map.table_walls_start_x = 0
        self.map.table_walls_end_x = self.AN_IMAGE_WIDTH
        self.map.table_walls_start_y = 0
        self.map.table_walls_end_y = self.AN_IMAGE_HEIGHT
        width = 1

        self.map.add_left_wall(width)

        column = [self.map.node_matrix[i][0] for i in range(len(self.map.node_matrix))]
        assert all([node.role is TileRole.OBSTACLE for node in column])

    def test_when_add_right_wall_then_there_are_obstacles_to_the_right(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        self.map.table_walls_start_x = 0
        self.map.table_walls_end_x = self.AN_IMAGE_WIDTH
        self.map.table_walls_start_y = 0
        self.map.table_walls_end_y = self.AN_IMAGE_HEIGHT
        width = 1

        self.map.add_right_wall(width)

        column = [self.map.node_matrix[i][-1] for i in range(len(self.map.node_matrix))]
        assert all([node.role is TileRole.OBSTACLE for node in column])

    def test_when_create_round_obstacle_then_the_obstacle_is_round(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        node = self.map.get_node_from_matrix_coordinates((5, 5))
        width = 2*self.map.node_size
        role = TileRole.OBSTACLE
        _uuid = uuid.uuid4()
        x, y = node.pixel_coordinates_center

        self.map.create_round_obstacle((int(x), int(y)), width, role, _uuid)

        expected_obstacle_positions = {
            (5, 5), (4, 4), (5, 5),
            (4, 4), (6, 4), (4, 6), (6, 6),
            (4, 5), (3, 5),
            (6, 5), (5, 3),
            (5, 4), (5, 6)

        }
        found_obstacle_positions = set()
        for i in range(len(self.map.node_matrix)):
            for j in range(len(self.map.node_matrix[0])):
                node = self.map.node_matrix[i][j]
                if node.role is TileRole.OBSTACLE or node.role is TileRole.CUSHION:
                    found_obstacle_positions.add(node.matrix_center)
        assert expected_obstacle_positions == found_obstacle_positions

    def test_when_create_square_obstacle_then_the_obstacle_is_square(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        node = self.map.get_node_from_matrix_coordinates((5, 5))
        width = 2*self.map.node_size
        role = TileRole.OBSTACLE
        _uuid = uuid.uuid4()
        x, y = node.pixel_coordinates_center

        self.map.create_square_obstacle((int(x), int(y)), width, role, _uuid)

        expected_obstacle_positions = {
            (5, 5), (6, 3), (3, 6), (4, 3),
            (4, 4), (6, 4), (4, 6), (6, 6),
            (4, 5), (3, 3), (3, 4),
            (6, 5), (3, 5), (5, 3),
            (5, 4), (5, 6)
        }
        found_obstacle_positions = set()
        for i in range(len(self.map.node_matrix)):
            for j in range(len(self.map.node_matrix[0])):
                node = self.map.node_matrix[i][j]
                if node.role is TileRole.OBSTACLE or node.role is TileRole.CUSHION:
                    found_obstacle_positions.add(node.matrix_center)
        assert expected_obstacle_positions == found_obstacle_positions

    def test_when_set_obstacle_then_an_obstacle_is_set(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        node = self.map.get_node_from_pixel((30, 30))
        x, y = node.pixel_coordinates_center

        self.map.set_obstacle((int(x), int(y)))

        assert node.role is TileRole.OBSTACLE

    def test_when_set_puck_then_a_puck_is_set(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        node = self.map.get_node_from_pixel((30, 30))
        x, y = node.pixel_coordinates_center

        self.map.set_puck((int(x), int(y)))

        assert node.role is TileRole.PUCK

    def test_when_add_table_walls_then_there_are_obstacles_all_around(self):
        self.map.create_nodes()
        self.map.connect_nodes()
        self.map.table_walls_start_x = 0
        self.map.table_walls_end_x = self.AN_IMAGE_WIDTH
        self.map.table_walls_start_y = 0
        self.map.table_walls_end_y = self.AN_IMAGE_HEIGHT
        self.map.robot_width = 1
        self.map.safety_cushion = 0

        self.map.add_table_walls()

        first_column = [self.map.node_matrix[i][0] for i in range(len(self.map.node_matrix))]
        last_column = [self.map.node_matrix[i][-1] for i in range(len(self.map.node_matrix))]
        assert all([node.role is TileRole.OBSTACLE for node in self.map.node_matrix[0]])
        assert all([node.role is TileRole.OBSTACLE for node in self.map.node_matrix[-1]])
        assert all([node.role is TileRole.OBSTACLE for node in first_column])
        assert all([node.role is TileRole.OBSTACLE for node in last_column])
