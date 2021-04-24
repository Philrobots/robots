import cv2

import math

from detection.acuro_markers.obstacle_and_robot_finder import \
    ObstacleRobotFinder
from detection.puck_detection import PuckDetection
from detection.hard_code_position.square_corner_detection import \
    SquareCornerDetection

PIXEL_TO_CM = 6.882391855


robot_and_obstacle_finder = ObstacleRobotFinder()
puck_finder = PuckDetection()
square_corner_detection = SquareCornerDetection()


def distance_item(item1, item2):
    (x1, y1), item_type1 = item1
    (x2, y2), item_type2 = item2

    dist = distance(x1, y1, x2, y2)

    if item_type1 == "puck":
        dist -= 25
    elif item_type1 == "corner":
        dist -= 10
    elif item_type1 == "wall":
        pass
    elif item_type1 == "obstacle":
        dist -= 42.5

    if item_type2 == "puck":
        dist -= 25
    elif item_type2 == "corner":
        dist -= 10
    elif item_type2 == "wall":
        pass
    elif item_type2 == "obstacle":
        dist -= 42.5

    return dist / PIXEL_TO_CM


def get_edge_position(item, angle):
    (x1, y1), item_type = item

    if item_type == "puck":
        radius = 25
    elif item_type == "corner":
        radius = 10
    elif item_type == "wall":
        radius = 0
    elif item_type == "obstacle":
        radius = 42.5
    else:
        raise Exception(f"item_type pas dans puck, wall, corner, obstacle. pas suppose. item_type={item_type}")

    x1 = x1 + math.cos(angle)*radius
    y1 = y1 - math.sin(angle)*radius
    return x1, y1


def get_angle_between_two_positions(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return -math.atan2(y2 - y1, x2 - x1)


def distance(x1, y1, x2, y2):
    return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))


def draw_line(image, position1, position2, color):
    x1, y1 = position1
    x2, y2 = position2

    if color == "red":
        line_color = (0, 0, 255)
    elif color == "green":
        line_color = (0, 255, 0)
    else:
        line_color = (255, 255, 255)
    line_thickness = 2
    cv2.line(image, (int(x1), int(y1)), (int(x2), int(y2)), line_color, thickness=line_thickness)


def write_text(image, text, bottom_left_corner_of_text):
    x, y = bottom_left_corner_of_text
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_color = (255, 255, 255)
    line_type = 2
    cv2.putText(image, text, (int(x), int(y)), font, font_scale, font_color, line_type)


def take_new_reference_image_from_video():
    video_path = "deplacement_table_de_jeux.mkv"
    cap = cv2.VideoCapture(video_path)
    ret, frame = cap.read()
    cv2.imwrite("frame.jpg", frame)
    cv2.imshow("", frame)
    cv2.waitKey(0)


def take_new_reference_image_from_camera_stream():
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 904)
    ret, frame = cap.read()
    cv2.imwrite("frame.jpg", frame)
    cv2.imshow("", frame)
    cv2.waitKey(0)


def write_distances_on_image(image, walls):
    pucks_dict = puck_finder.detect_pucks(image)
    obstacles_list = robot_and_obstacle_finder.detect_obstacle_position(image, DEBUG=False)
    corners_dict = square_corner_detection.detect_square_position(image, DEBUG=False)

    pucks = [(tuple(single["center_position"]), "puck") for color in pucks_dict for single in pucks_dict[color]]
    obstacles = [(tuple(single["center_of_obstacle"]), "obstacle") for single in obstacles_list]
    corners = [(tuple(corners_dict[corner]), "corner") for corner in ["corner_A", "corner_B", "corner_C", "corner_D"]]

    all_items = pucks + obstacles + corners

    TOP_WALL_Y, BOT_WALL_Y, RIGHT_WALL_X, LEFT_WALL_X = walls
    for i, item in enumerate(all_items):

        #add walls
        wall_positions = []
        up = ((item[0][0], TOP_WALL_Y), "wall")
        left = ((LEFT_WALL_X, item[0][1]), "wall")
        down = ((item[0][0], BOT_WALL_Y), "wall")
        right = ((RIGHT_WALL_X, item[0][1]), "wall")

        wall_positions.append(up)
        wall_positions.append(left)
        wall_positions.append(down)
        wall_positions.append(right)

        # calculate distances
        other_items = all_items[:i] + all_items[i + 1:] + wall_positions
        distance_from_item = [(distance_item(other_item, item), i) for i, other_item in enumerate(other_items)]
        distance_from_item = [other_item for other_item in distance_from_item if other_item[0] <= 20]

        for dist, i in distance_from_item:
            other_item = other_items[i]
            other_item, other_item_type = other_item

            # Pour que la ligne parte de la bordure de l'objet
            angle_from_item = get_angle_between_two_positions(item[0], other_item)
            angle_from_other_item = get_angle_between_two_positions(other_item, item[0])
            item_edge_position = get_edge_position(item, angle_from_item)
            other_item_edge_position = get_edge_position((other_item, other_item_type), angle_from_other_item)
            assert int(distance(*item_edge_position, *other_item_edge_position)/PIXEL_TO_CM) == int(dist)

            if dist < 10:
                draw_line(image, item_edge_position, other_item_edge_position, "red")
            else:
                draw_line(image, item_edge_position, other_item_edge_position, "green")

            middle_y = min(other_item[1], item[0][1])
            middle_x = min(other_item[0], item[0][0])
            halfway_x = int(middle_x + abs(other_item[0] - item[0][0]) / 2)
            halfway_y = int(middle_y + abs(other_item[1] - item[0][1]) / 2)
            write_text(image, str(dist)[:5], (halfway_x, halfway_y))


def verify_distance_on_live_stream():
    TOP_WALL_Y = 45
    BOT_WALL_Y = 808
    RIGHT_WALL_X = 1581
    LEFT_WALL_X = 10
    walls = [TOP_WALL_Y, BOT_WALL_Y, RIGHT_WALL_X, LEFT_WALL_X]

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 904)

    while True:
        ret, image = cap.read()
        while image is None:
            ret, image = cap.read()

        write_distances_on_image(image, walls)

        cv2.imshow('', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def verify_distances_on_image(image_path):
    TOP_WALL_Y = 45
    BOT_WALL_Y = 808
    RIGHT_WALL_X = 1581
    LEFT_WALL_X = 10
    walls = [TOP_WALL_Y, BOT_WALL_Y, RIGHT_WALL_X, LEFT_WALL_X]
    image = cv2.imread(image_path)

    write_distances_on_image(image, walls)
    cv2.imshow('', image)
    cv2.waitKey(0)


def verify_distances_on_video(video_path):
    TOP_WALL_Y = 92
    BOT_WALL_Y = 920
    RIGHT_WALL_X = 1780
    LEFT_WALL_X = 73
    walls = [TOP_WALL_Y, BOT_WALL_Y, RIGHT_WALL_X, LEFT_WALL_X]

    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 904)

    while True:
        ret, image = cap.read()
        while image is None:
            ret, image = cap.read()

        write_distances_on_image(image, walls)

        cv2.imshow('', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    video_path = "deplacement_table_de_jeux.mkv"
    #verify_distances_on_video(video_path)

    #image_path = "monde8.jpg"
    #image_path = "WIN_20210301_14_58_08_Pro.jpg"
    #image_path = "WIN_20210302_12_53_39_Pro.jpg"
    #verify_distances_on_image(image_path)

    verify_distance_on_live_stream()
    #take_new_reference_image_from_camera_stream()





