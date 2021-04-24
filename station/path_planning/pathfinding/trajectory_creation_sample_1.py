"""Example of how to find a path between the robot and a specific puck
given the obstacles, a start position, an end position and where the other pucks are."""
from PIL import Image

from scripts.src.pathfinding.show_path import get_path


if __name__ == '__main__':
    NODE_SIZE = 25
    NODE_IDENTIFIER_WIDTH = NODE_SIZE / 5
    ALGORITHM = "BreadthFirstSearch"

    OBSTACLES = [
        (1142, 290),
        (657, 761),
    ]

    START = (1048, 504)
    END = (526, 418)

    PUCKS = [
        (241, 288),
        (373, 277),
        (332, 515),
        (250, 588),
        (807, 341),
        (800, 435),
        (745, 787),
        (1093, 655),
        (1150, 215)
    ]

    IMAGE_PATH = "./scripts/data/images/trajectory_example_1.jpg"
    IMAGE_WIDTH, IMAGE_HEIGHT = Image.open(IMAGE_PATH).size
    print(get_path(NODE_SIZE, ALGORITHM, OBSTACLES, START, END, PUCKS, IMAGE_WIDTH, IMAGE_HEIGHT))
