from dataclasses import dataclass
import numpy as np


@dataclass
class CameraCalibration:
    image_width: int
    image_height: int
    aspect_ratio: float
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    error: float

    def to_new_dimensions(self, new_image_width, new_image_height):
        new_camera_calibration = CameraCalibration(**self.__dict__.copy())

        new_camera_calibration.camera_matrix[0] *= new_image_width / self.image_width
        new_camera_calibration.camera_matrix[1] *= new_image_height / self.image_height

        new_camera_calibration.image_width = new_image_width
        new_camera_calibration.image_height = new_image_height
        new_camera_calibration.aspect_ratio = new_image_width / new_image_height

        return new_camera_calibration
