import cv2
import numpy as np

from detection.lower_boundary import LowerBoundary
from detection.upper_boundary import UpperBoundary


class ObjectDetection:

    def __init__(self, image, object_to_detect, minimum_dimension, maximum_dimension):
        self.image = cv2.imread(image)
        self.lower_boundary = LowerBoundary()
        self.upper_boundary = UpperBoundary()
        self.object_to_detect = object_to_detect
        self.object_minimum_dimension = minimum_dimension
        self.object_maximum_dimension = maximum_dimension

    def copy_image(self):
        try:
            image = self.image.copy()
        except AttributeError as invalid_image:
            raise AttributeError("L'image est invalide") from invalid_image
        return image

    def generate_puck_position(self, x_position, y_position, width, height):
        return {
            "x_position": x_position,
            "y_position": y_position,
            "width": width,
            "height": height
        }

    def _destroy_windows(self):
        while 1:
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

    def _show_image(self, image_copy):
        cv2.imshow("Color detection", np.hstack([image_copy]))
        cv2.waitKey(50000)

    def draw_rectangle_on_image(self, image_copy, x_position, y_position, width,
                                height, object_type):
        cv2.rectangle(image_copy, (x_position, y_position), (x_position + width,
                                                             y_position + height), (0, 255, 0), 2)
        cv2.putText(image_copy, object_type, (x_position + (width // 2) - 30,
                                              y_position + (height // 3) - 30),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)

    def object_is_in_range(self, width, height):
        return self.object_minimum_dimension < width < self.object_maximum_dimension and \
               self.object_minimum_dimension < height < self.object_maximum_dimension
