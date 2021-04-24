from scripts.src.processing.process_image_to_grayscale import process_image_to_grayscale
from scripts.tests.util.command_panel_helper import command_panel_images

images = command_panel_images


def test_given_image_then_grayscale_is_valid():
    for image in images:
        assert_grayscale_is_valid(image)


def assert_grayscale_is_valid(image):
    image = process_image_to_grayscale(image)

    assert image is not None
