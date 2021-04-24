from scripts.src.capture.capture_image_from_path import capture_image_from_path

command_panel_images_path = [
    'data/images/command_panel_example_1.png',
    'data/images/command_panel_example_2.png'
]

command_panel_images = []

for path in command_panel_images_path:
    command_panel_images.append(capture_image_from_path(path))

command_panel_images_letters = [
    ['A', 'B', 'D', 'C', 'B', 'C', 'A', 'D', 'A'],
    ['C', 'B', 'A', 'B', 'A', 'C', 'D', 'A', 'D']
]
