from pathfinding.pathfinder import Pathfinder
from pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from pathfinding.map import Map


def get_path(node_size, algorithm, obstacles, start, end, pucks, image_width, image_height):
    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)
    board_map = Map(image_width, image_height, obstacles, pucks, start, end, node_size=node_size)
    board_map.render_map()
    pathfinder = Pathfinder(board_map, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    return pathfinder.path


def get_path_and_map(node_size, algorithm, obstacles, start, end, pucks, image_width, image_height):
    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)
    board_map = Map(image_width, image_height, obstacles, pucks, start, end, node_size=node_size)
    board_map.render_map()
    pathfinder = Pathfinder(board_map, pathfinding_algorithm)

    pathfinder.find_square_matrix_path()
    return pathfinder.path, board_map
