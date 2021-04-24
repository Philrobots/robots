from math import pi
from scripts.src.detection.position_calculator import PositionCalculator


A_POINT = (700, 300)
ANOTHER_POINT = (900, 450)
ANGLE_POINT = (2, 2)
ANOTHER_ANGLE_POINT = (4, 2)

position_calculator = PositionCalculator()


def test_given_two_point_when_calculate_distance_in_x_should_return_right_distance():
    actual_x_distance_between_point = position_calculator.calculate_difference_of_x(A_POINT, ANOTHER_POINT)

    expected_x_distance_between_point = 200

    assert actual_x_distance_between_point == expected_x_distance_between_point


def test_given_two_point_when_calculate_distance_in_y_should_return_right_distance():
    actual_y_distance_between_point = position_calculator.calculate_difference_of_y(A_POINT, ANOTHER_POINT)

    expected_y_distance_between_point = -150

    assert actual_y_distance_between_point == expected_y_distance_between_point


def test_given_two_point_when_calculate_angle_between_two_point_should_return_a_positive_angle():
    actual_angle = position_calculator.calculate_angle_between_two_position(A_POINT, ANOTHER_POINT)

    assert actual_angle >= 0


def test_given_two_point_when_calculate_angle_between_two_point_should_return_angle_lower_than_2_pi():
    actual_angle = position_calculator.calculate_angle_between_two_position(A_POINT, ANOTHER_POINT)

    assert actual_angle <= pi * 2


def test_given_two_point_when_calculate_angle_between_two_point_should_return_the_right_angle():
    actual_angle = position_calculator.calculate_angle_between_two_position(ANGLE_POINT, ANOTHER_ANGLE_POINT)

    expected_angle = 0

    assert actual_angle == expected_angle
