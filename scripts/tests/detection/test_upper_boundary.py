from scripts.src.detection.upper_boundary import UpperBoundary

RED_COLOR = "red"
BLUE_COLOR = "blue"
YELLOW_COLOR = "yellow"
NOT_A_COLOR = "Hello world"

upper_boundary = UpperBoundary()


def test_given_red_color_then_should_return_right_upper_boundary():
    expected_boundaries = [179, 255, 106]

    actual_boundaries = upper_boundary.get_upper_boundaries(RED_COLOR)

    assert len(actual_boundaries) == len(expected_boundaries)
    assert all([a == b for a, b in zip(actual_boundaries, expected_boundaries)])


def test_given_blue_color_then_should_return_right_lower_boundary():
    expected_boundaries = [179, 255, 255]

    actual_boundaries = upper_boundary.get_upper_boundaries(BLUE_COLOR)

    assert len(actual_boundaries) == len(expected_boundaries)
    assert all([a == b for a, b in zip(actual_boundaries, expected_boundaries)])


def test_given_yellow_color_then_should_return_right_upper_boundary():
    expected_boundaries = [41, 255, 255]

    actual_boundaries = upper_boundary.get_upper_boundaries(YELLOW_COLOR)

    assert len(actual_boundaries) == len(expected_boundaries)
    assert all([a == b for a, b in zip(actual_boundaries, expected_boundaries)])


def test_given_not_a_valid_color_then_should_return_array_of_zeros():
    expected_boundaries = [0, 0, 0]

    actual_boundaries = upper_boundary.get_upper_boundaries(NOT_A_COLOR)

    assert len(actual_boundaries) == len(expected_boundaries)
    assert all([a == b for a, b in zip(actual_boundaries, expected_boundaries)])
