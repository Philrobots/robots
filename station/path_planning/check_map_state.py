import cv2
import numpy as np

from pathfinding.pathfinder import Pathfinder
from pathfinding.pathfinding_algorithm_factory import PathfindingAlgorithmFactory
from pathfinding.map import Map
from pathfinding.map_drawer import MapDrawer
from pathfinding.pathfinding_algorithms import PathfindingAlgorithms
from pathfinding.path_not_found_exception import PathNotFoundException
from pathfinding.tile_role import TileRole

import sys
sys.path.append("../robot_goal_and_obstacle_finder")
from detection.acuro_markers.obstacle_and_robot_finder import \
    ObstacleRobotFinder
from detection.puck_detection import PuckDetection


node_size = 15
node_identifier_width = node_size // 5
robot_and_obstacle_finder = ObstacleRobotFinder()
puck_finder = PuckDetection()


def get_map_and_pathfinder(node_size, algorithm, obstacles, start, end, pucks, image_width, image_height):
    pathfinding_algorithm_factory = PathfindingAlgorithmFactory()
    pathfinding_algorithm = pathfinding_algorithm_factory.create(algorithm)
    board_map = Map(image_width, image_height, obstacles, pucks, start, end, node_size=node_size)
    board_map.render_map()
    pathfinder = Pathfinder(board_map, pathfinding_algorithm)
    return pathfinder, board_map


def check_map_state_on_image(image, color, position):
    height, width, channels = image.shape
    pucks_dict = puck_finder.detect_pucks(image)
    obstacles_list = robot_and_obstacle_finder.detect_obstacle_position(image, DEBUG=False)
    center_of_bottom_robot, prehenseur_position, angle_robot = robot_and_obstacle_finder.detect_robot(image)

    pucks = [single["center_position"] for color in pucks_dict for single in pucks_dict[color]]
    obstacles = [single["center_of_obstacle"] for single in obstacles_list]

    start = center_of_bottom_robot

    if color:
        end = pucks_dict[color][0]["center_position"]
    elif position:
        end = position
    else:
        end = (500, 500)

    pucks = [puck for puck in pucks if puck != end]

    pathfinder, board_map = get_map_and_pathfinder(node_size, PathfindingAlgorithms.BREADTH_FIRST_SEARCH, obstacles,
                                                   start, end, pucks, width, height)
    map_drawer = MapDrawer(node_identifier_width, node_size, image)

    try:
        pathfinder.find_square_matrix_path()
        path = pathfinder.path
    except PathNotFoundException:
        print("path not found")
        if board_map.get_node_from_pixel(start).role is not TileRole.EMPTY:
            print("the robot is inside an obstacle")
        elif board_map.get_node_from_pixel(end).role is not TileRole.EMPTY:
            print("the goal is inside an obstacle")
        else:
            print("there's really no path between the two")
        path = []

    map_drawer.draw_map(board_map, path)
    image = map_drawer.get_image()

    open_cv_image = np.array(image.convert('RGB'))
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    return open_cv_image


def check_map_state_on_a_stream(color, position):
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 904)

    while True:
        ret, image = cap.read()
        while image is None:
            ret, image = cap.read()

        image = check_map_state_on_image(image, color, position)

        cv2.imshow('', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    color = "yellow"
    position = None
    #color = None
    #position = (400, 400)

    #image_path = "WIN_20210302_12_53_39_Pro.jpg"
    #image = cv2.imread(image_path)
    #image = check_map_state_on_image(image, color, position)
    #cv2.imshow('', image)
    #cv2.waitKey(0)


    check_map_state_on_a_stream(color, position)

