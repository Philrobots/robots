import pytest

from scripts.src.capture.capture_image_from_path import capture_image_from_path
from scripts.tests.util.command_panel_helper import command_panel_images_path


def test_given_valid_path_then_image_is_valid():
    valid_paths = command_panel_images_path

    for valid_path in valid_paths:
        assert_image_is_valid(valid_path)


def assert_image_is_valid(path):
    image = capture_image_from_path(path)

    assert image is not None


def test_given_invalid_path_then_raise_exception():
    invalid_path = 'data/images/invalid.png'

    with pytest.raises(Exception):
        capture_image_from_path(invalid_path)
