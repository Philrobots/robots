"""
Helper class to facilitate debugging by drawing the generated map
as well as the path found by the algorithm
"""

from PIL import ImageDraw, Image

from pathfinding.tile_role import TileRole


class MapDrawer:
    """
    Helper class to facilitate debugging by drawing the generated map
    as well as the path found by the algorithm
    """
    def __init__(self, node_identifier_width, node_size, image):
        image = Image.fromarray(image)
        self.node_identifier_width = node_identifier_width
        self.node_size = node_size
        self.image = image
        self.width, self.height = self.image.size
        self.draw = ImageDraw.Draw(self.image)
        self.role_to_drawing_function = self.get_role_to_drawing_function()

    def _draw_line(self, first_point_x, first_point_y, second_point_x, second_point_y, color):
        """Draws a line from point A(x1,y1) to point B(x2,y2)"""
        self.draw.line((first_point_x, first_point_y, second_point_x, second_point_y), fill=color)

    def _draw_rectangle(self, node, color, outline=None):
        """Draws a rectangle on the given node"""
        x_position, y_position = node.pixel_coordinates_center
        self.draw.rectangle(
            [(x_position - self.node_identifier_width, y_position - self.node_identifier_width),
             (x_position + self.node_identifier_width, y_position + self.node_identifier_width)],
            fill=color, outline=outline)

    def get_role_to_drawing_function(self):
        """Maps the different node roles to which drawing function should be used to draw them"""
        return {
            TileRole.OBSTACLE: self.draw_obstacle,
            TileRole.CUSHION: self.draw_cushion,
            TileRole.PUCK: self.draw_puck,
            TileRole.START: self.draw_start_node,
            TileRole.END: self.draw_end_node,
            TileRole.EMPTY: lambda x: None
        }

    def draw_map(self, _map, path):
        """Draws the node matrix, the different obstacles and the path used to get from
        the starting node to the end node."""
        self.draw_board()

        for line in _map.get_node_matrix():
            for node in line:
                drawing_function = self.role_to_drawing_function[node.role]
                drawing_function(node)

        self.draw_path(path)

    def draw_board(self):
        """Draws the node matrix"""
        self.draw_vertical_lines()
        self.draw_horizontal_lines()

    def draw_vertical_lines(self):
        """Draws the vertical lines for the node matrix"""
        for i in range((self.width // self.node_size) + 1):
            first_point_x = i * self.node_size
            first_point_y = 0
            second_point_x = i * self.node_size
            second_point_y = self.height
            color = 128

            self._draw_line(first_point_x, first_point_y, second_point_x, second_point_y, color)

    def draw_horizontal_lines(self):
        """Draws the horizontal lines for the node matrix"""
        for i in range((self.height // self.node_size) + 1):
            first_point_x = 0
            first_point_y = i * self.node_size
            second_point_x = self.width
            second_point_y = i * self.node_size
            color = 128

            self._draw_line(first_point_x, first_point_y, second_point_x, second_point_y, color)

    def draw_cushion(self, node):
        """Draws the given node as a cushion on the image"""
        color = (0, 255, 0)
        self._draw_rectangle(node, color)

    def draw_obstacle(self, node):
        """Draws the given node as an obstacle on the image"""
        color = (255, 255, 255)
        self._draw_rectangle(node, color)

    def draw_puck(self, node):
        """Draws the given node as a puck on the image"""
        color = (0, 0, 255)
        self._draw_rectangle(node, color)

    def draw_start_node(self, node):
        """Draws the given node as a starting node on the image"""
        color = 200
        self._draw_rectangle(node, color)

    def draw_end_node(self, node):
        """Draws the given node as an end node on the image"""
        color = 120
        self._draw_rectangle(node, color)

    def draw_path(self, path):
        """Draws the path from the starting node to the end node"""
        for node in path:
            color = (0, 122, 9)
            outline = 300

            self._draw_rectangle(node, color, outline)

    def get_image(self):
        """Gets the image"""
        return self.image
