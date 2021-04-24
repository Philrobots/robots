from scripts.src.pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from scripts.src.pathfinding.map import Map


def get_path(node_size, algorithm, obstacles, start, end, pucks, image_width, image_height):
    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)

    board_map = Map(image_width, image_height, node_size=node_size)
    board_map.render_map()
    for obstacle in obstacles:
        board_map.set_obstacle(obstacle)
    for puck in pucks:
        board_map.set_puck(puck)

    path = pathfinding_algorithm.find_path(board_map.get_node_from_pixel(start), board_map.get_node_from_pixel(end))
    return path


def get_path_and_map(node_size, algorithm, obstacles, start, end, pucks, image_width, image_height):
    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)

    board_map = Map(image_width, image_height, node_size=node_size)
    board_map.render_map()
    for obstacle in obstacles:
        board_map.set_obstacle(obstacle)
    for puck in pucks:
        board_map.set_puck(puck)

    path = pathfinding_algorithm.find_path(board_map.get_node_from_pixel(start), board_map.get_node_from_pixel(end))
    return path, board_map
