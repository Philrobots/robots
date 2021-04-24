import os

import cv2
import numpy as np


class SquareDetection:

    def detect_square(self, image, Debug=True):
        script_dir = os.path.dirname(__file__)
        rel_path = image
        abs_file_path = os.path.join(script_dir, rel_path)
        image = cv2.imread(abs_file_path)
        image_copy = image.copy()

        image_canny = self.get_image_canny(image)

        kernel = np.ones((5,5))
        image_dilate = cv2.dilate(image_canny, kernel, iterations=1)
        square_corners = self.get_contours(image_dilate, image_copy)

        if Debug:
            self.show_image(image_copy)

        return square_corners

    def get_image_canny(self, image):
        image_blur = cv2.GaussianBlur(image, (7, 7), 1)
        image_gray = cv2.cvtColor(image_blur, cv2.COLOR_BGR2GRAY)
        image_canny = cv2.Canny(image_gray, 90, 90)
        return image_canny

    def show_image(self, image_output):
        cv2.imshow('square detection', image_output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    @staticmethod
    def generate_four_corners(x_position, y_position, width, height):
        corner_a = (x_position + width, y_position)
        corner_b = (x_position + width, y_position + height)
        corner_c = (x_position, y_position + height)
        corner_d = (x_position, y_position)

        return {
            "corner_A": corner_a,
            "corner_B": corner_b,
            "corner_C": corner_c,
            "corner_D": corner_d
        }


    def get_contours(self, image, image_copy):
        if image is None:
            return self.generate_four_corners(0, 0, 0, 0)

        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        four_corners = {}
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200000:
                cv2.drawContours(image_copy, contour, -1, (255, 255, 0), 2)
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
                x_position, y_position, width, height = cv2.boundingRect(approx)
                four_corners = self.generate_four_corners(x_position, y_position,
                                                          width, height)
                break
        return four_corners
