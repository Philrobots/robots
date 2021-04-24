import cv2

from scripts.src.detection.acuro_markers.obstacle_detection import ObstacleDetection

A_IMAGE = "robot_5x5_five.jpg"
A_INVALID_IMAGE = "invalid_image.jpg"

obstacle_detection = ObstacleDetection()

image = cv2.imread(A_IMAGE)


def test_given_a_valid_image_should_return_an_list():
    expected_position = obstacle_detection.detect_aruco_marker_on_obstacle(image)

    assert isinstance(expected_position, list)


def test_given_a_valid_image_should_return_a_list_of_2_items():
    expected_position = obstacle_detection.detect_aruco_marker_on_obstacle(image)

    assert len(expected_position) == 2


def test_given_a_valid_image_should_return_list_with_valid_fist_center():
    expected_position = obstacle_detection.detect_aruco_marker_on_obstacle(image)

    assert isinstance(expected_position, list)
    assert len(expected_position) == 2


def test_generate_empty_obstacle_position():
    expected_result = obstacle_detection.generate_empty_obstacle_position()
    obstacle_1 = expected_result[0]["obstacle 1"]
    obstacle_2 = expected_result[1]["obstacle 2"]

    assert isinstance(obstacle_1, dict)
    assert isinstance(expected_result, list)
    assert len(obstacle_1) == 5
    assert len(obstacle_2) == 5
    assert obstacle_1["center"] == (0, 0)
