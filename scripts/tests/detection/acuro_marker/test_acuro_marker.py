from scripts.src.detection.acuro_markers.AcuroMarkers import ArucoMarkers


A_POSITION = (2, 2)
ANOTHER_POSITION = (4, 4)
aruco_marker = ArucoMarkers()
INVALID_IMAGE = "invalid.png"


def test_given_two_position_then_should_return_the_right_center_x_position():
    actual_center_x, actual_center_y = aruco_marker.\
        generate_center_position(A_POSITION, ANOTHER_POSITION)

    expected_center_x = 3

    assert actual_center_x == expected_center_x


def test_given_two_position_then_should_return_the_right_center_y_position():
    actual_center_x, actual_center_y = aruco_marker.\
        generate_center_position(A_POSITION, ANOTHER_POSITION)

    expected_center_y = 3

    assert actual_center_y == expected_center_y
