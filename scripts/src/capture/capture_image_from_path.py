import os
import cv2


def capture_image_from_path(path):
    absolute_path = os.path.join(os.getcwd(), path)

    image = cv2.imread(absolute_path)

    if image is None:
        raise Exception(f'Image not found from path : {absolute_path}')

    return image
