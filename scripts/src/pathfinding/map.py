"""
Class that represents where the robot can move and where the
different obstacles and objects laying on the table are.
"""
import math
import uuid

from scripts.src.pathfinding.node import Node
from scripts.src.pathfinding.tile_role import TileRole
from scripts.src.pathfinding.direction import Direction
from scripts.src.pathfinding.obstacle_representation import ObstacleRepresentation


class Map:
    """
    Class that represents where the robot can move and where the
    different obstacles and objects laying on the table are.
    """
    def __init__(self, image_width, image_height, node_size=25,
                 safety_cushion=0, robot_width=100, obstacle_width=40, puck_width=25,
                 obstacle_representation=ObstacleRepresentation.SQUARE):
        self.node_size = node_size
        self.safety_cushion = safety_cushion
        self.robot_width = robot_width
        self.obstacle_width = obstacle_width
        self.puck_width = puck_width
        self.obstacle_cushion_width = self.safety_cushion + self.robot_width + self.obstacle_width
        self.obstacle_puck_width = self.safety_cushion + self.robot_width + self.puck_width
        self.obstacle_representation = obstacle_representation

        self.width, self.height = image_width, image_height

        self.obstacles = []
        self.pucks = []

        self.node_matrix = []

        self.table_walls_start_y = 40
        self.table_walls_end_y = 810
        self.table_walls_start_x = 0
        self.table_walls_end_x = 1600

    def render_map(self):
        """Creates the nodes and generates the obstacles, pucks, start and end node."""
        self.create_nodes()
        self.connect_nodes()
        self.add_table_walls()

    def add_top_wall(self, width):
        top_wall_uuid = uuid.uuid4()
        start_wall_top = max(0, self.table_walls_start_y // self.node_size)
        for row in range(
                start_wall_top,
                ((self.table_walls_start_y + width) // self.node_size) + 1):
            for node in self.node_matrix[row]:
                node.role = TileRole.OBSTACLE
                node.uuid = top_wall_uuid
                node.held_by.add(top_wall_uuid)

    def add_bottom_wall(self, width):
        bot_wall_uuid = uuid.uuid4()
        end_wall_bot = min(len(self.node_matrix), (self.table_walls_end_y // self.node_size)+1)
        for row in range(
                (self.table_walls_end_y - width) // self.node_size,
                end_wall_bot):
            for node in self.node_matrix[row]:
                node.role = TileRole.OBSTACLE
                node.uuid = bot_wall_uuid
                node.held_by.add(bot_wall_uuid)

    def add_right_wall(self, width):
        right_wall_uuid = uuid.uuid4()
        end_wall_right = min(len(self.node_matrix[0]), (self.table_walls_end_x//self.node_size)+1)
        for column in range(
                (self.table_walls_end_x - width) // self.node_size,
                end_wall_right):
            for row in range(len(self.node_matrix)):
                node = self.get_node_from_matrix_coordinates((column, row))
                node.role = TileRole.OBSTACLE
                node.uuid = right_wall_uuid
                node.held_by.add(right_wall_uuid)

    def add_left_wall(self, width):
        left_wall_uuid = uuid.uuid4()
        start_wall_left = max(0, (self.table_walls_start_x // self.node_size))
        for column in range(
                start_wall_left,
                ((self.table_walls_start_x + width)//self.node_size)+1):
            for row in range(len(self.node_matrix)):
                node = self.get_node_from_matrix_coordinates((column, row))
                node.role = TileRole.OBSTACLE
                node.uuid = left_wall_uuid
                node.held_by.add(left_wall_uuid)

    def add_table_walls(self):
        width = self.robot_width + self.safety_cushion
        self.add_top_wall(width)
        self.add_bottom_wall(width)
        self.add_left_wall(width)
        self.add_right_wall(width)

    def create_nodes(self):
        """Creates the matrix containing the nodes."""
        node_matrix = [
            [] for _ in range((self.height // self.node_size) + 1)
        ]

        for column in range((self.height // self.node_size)+1):
            for row in range((self.width // self.node_size)+1):
                y_position = column * self.node_size + self.node_size / 2
                x_position = row * self.node_size + self.node_size / 2
                node_matrix[column].append(
                    Node((column, row), (x_position, y_position), self.node_size, self.node_size))

        self.node_matrix = node_matrix

    def connect_nodes(self):
        """Connect each node to its neighbors. This method basically
        defines what movements are allowed by the robot."""
        for line in self.node_matrix:
            for node in line:
                possible_neighbors = [
                    (y_position, x_position, direction)

                    for (y_position, x_position, direction) in
                    [
                        (
                            node.matrix_center[0] - 1,
                            node.matrix_center[1],
                            Direction.UP
                        ),
                        (
                            node.matrix_center[0] + 1,
                            node.matrix_center[1],
                            Direction.DOWN
                        ),
                        (
                            node.matrix_center[0],
                            node.matrix_center[1] - 1,
                            Direction.LEFT
                        ),
                        (
                            node.matrix_center[0],
                            node.matrix_center[1] + 1,
                            Direction.RIGHT
                        ),

                        (
                            node.matrix_center[0] - 1,
                            node.matrix_center[1] - 1,
                            Direction.TOP_LEFT
                        ),
                        (
                            node.matrix_center[0] - 1,
                            node.matrix_center[1] + 1,
                            Direction.TOP_RIGHT
                        ),
                        (
                            node.matrix_center[0] + 1,
                            node.matrix_center[1] - 1,
                            Direction.DOWN_LEFT),
                        (
                            node.matrix_center[0] + 1,
                            node.matrix_center[1] + 1,
                            Direction.DOWN_RIGHT
                        ),
                    ]

                    if ((0 <= x_position < len(self.node_matrix[0])
                         and 0 <= y_position < len(self.node_matrix))
                        and (x_position, y_position) != node.matrix_center)
                ]

                for (y_position, x_position, direction) in possible_neighbors:
                    node.neighbors.append((self.node_matrix[y_position][x_position], direction))

    def delete_object(self, object_position: (int, int)):
        #TODO: pourrait chercher dans un carré autour de l'emplacement de la puck
        # au lieu de toute la matrix
        puck = self.get_node_from_pixel(object_position)
        _uuid = puck.uuid

        if _uuid is not None:
            for row in self.node_matrix:
                for node in row:
                    if _uuid in node.held_by:
                        node.held_by.remove(node.uuid)
                        if len(node.held_by) == 0:
                            node.uuid = None
                            node.role = TileRole.EMPTY
                        else:
                            node.uuid = list(node.held_by)[0]
        elif _uuid is None:
            raise Exception("t'essaies de delete une case vide")

    def create_round_obstacle(self, obstacle: (int, int), radius, role, obstacle_uuid):
        width, height = obstacle
        lower_range_column = int(max(0, ((height - radius) // self.node_size)))
        lower_range_row = int(max(0, ((width - radius) // self.node_size)))
        higher_range_column = int(min(len(self.node_matrix),
                                      ((height + radius) // self.node_size) + 1))
        higher_range_row = int(min(len(self.node_matrix[0]),
                                   ((width + radius) // self.node_size) + 1))

        for column in range(lower_range_column, higher_range_column):
            for row in range(lower_range_row, higher_range_row):
                node = self.get_node_from_matrix_coordinates((row, column))

                distance = get_distance(obstacle, node.pixel_coordinates_center)

                if distance < radius:
                    node.role = TileRole.CUSHION
                    node.uuid = obstacle_uuid
                    node.held_by.add(obstacle_uuid)

        obstacle_node = self.get_node_from_pixel(obstacle)
        obstacle_node.role = role
        obstacle_node.uuid = obstacle_uuid
        obstacle_node.held_by.add(obstacle_uuid)

    def create_square_obstacle(self, obstacle: (int, int), length, role, obstacle_uuid):
        width, height = obstacle
        lower_range_column = int(max(0, ((height - length) // self.node_size)))
        lower_range_row = int(max(0, ((width - length) // self.node_size)))
        higher_range_column = int(min(len(self.node_matrix), ((height + length) // self.node_size)))
        higher_range_row = int(min(len(self.node_matrix[0]), ((width + length) // self.node_size)))


        for column in range(lower_range_column, higher_range_column):
            for row in range(lower_range_row, higher_range_row):
                node = self.get_node_from_matrix_coordinates((row, column))
                node.role = TileRole.CUSHION
                node.uuid = obstacle_uuid
                node.held_by.add(obstacle_uuid)

        obstacle_node = self.get_node_from_pixel(obstacle)
        obstacle_node.role = role
        obstacle_node.uuid = obstacle_uuid
        obstacle_node.held_by.add(obstacle_uuid)

    def set_obstacle(self, obstacle):
        obstacle_uuid = uuid.uuid4()
        if self.obstacle_representation is ObstacleRepresentation.RADIUS:
            self.create_round_obstacle(obstacle, self.obstacle_cushion_width, TileRole.OBSTACLE, obstacle_uuid)
        elif self.obstacle_representation is ObstacleRepresentation.SQUARE:
            self.create_square_obstacle(obstacle, self.obstacle_cushion_width, TileRole.OBSTACLE, obstacle_uuid)
        else:
            self.create_round_obstacle(obstacle, self.obstacle_cushion_width, TileRole.OBSTACLE, obstacle_uuid)

    def set_puck(self, puck):
        obstacle_uuid = uuid.uuid4()
        if self.obstacle_representation is ObstacleRepresentation.RADIUS:
            self.create_round_obstacle(puck, self.obstacle_puck_width, TileRole.PUCK, obstacle_uuid)
        elif self.obstacle_representation is ObstacleRepresentation.SQUARE:
            self.create_square_obstacle(puck, self.obstacle_puck_width, TileRole.PUCK, obstacle_uuid)
        else:
            self.create_round_obstacle(puck, self.obstacle_puck_width, TileRole.PUCK, obstacle_uuid)

    def get_node_matrix(self):
        """Gets the node matrix"""
        return self.node_matrix

    def get_node_from_pixel(self, pixel):
        """Gets a node using any of the pixels the node should cover"""
        x_position = pixel[0] // self.node_size
        y_position = pixel[1] // self.node_size
        return self.node_matrix[y_position][x_position]

    def get_node_from_matrix_coordinates(self, coordinates):
        """Gets a node using its position in the node matrix"""
        x_position, y_position = coordinates
        return self.node_matrix[y_position][x_position]


def get_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
