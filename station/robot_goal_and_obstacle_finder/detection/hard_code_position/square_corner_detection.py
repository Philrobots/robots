import cv2
import numpy as np


class SquareCornerDetection:

    def __init__(self):
        self.position_of_corner_a = [600, 226]
        self.position_of_corner_b = [608, 645]
        self.position_of_corner_c = [185, 636]
        self.position_of_corner_d = [181, 226]
        self.position_of_center_square = [397, 430]

    def detect_square_position(self, image, DEBUG=False):
        image_in_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners_in_image = cv2.goodFeaturesToTrack(image_in_gray, 100, 0.01, 10)
        corners = np.int0(corners_in_image)
        corners_in_image = {
            "corner_A": [],
            "corner_B": [],
            "corner_C": [],
            "corner_D": []
        }

        for corner in corners:
            corner_x, corner_y = corner.ravel()
            if 150 <= corner_x <= 700 and 200 <= corner_y <= 750:
                cv2.circle(image, (corner_x, corner_y), 3, (0, 0, 255), -1)
                if 450 < corner_x < 700 and 150 < corner_y < 350:
                    corners_in_image["corner_A"].append((corner_x, corner_y))
                elif 450 < corner_x < 700 and 450 < corner_y < 750:
                    corners_in_image["corner_B"].append((corner_x, corner_y))
                elif 150 < corner_x < 300 and 450 < corner_y < 750:
                    corners_in_image["corner_C"].append((corner_x, corner_y))
                elif 150 < corner_y < 300 and 150 < corner_y < 350:
                    corners_in_image["corner_D"].append((corner_x, corner_y))

        position_a, position_b, position_c, position_d = \
            self.calculate_point_in_the_middle_for_each_corner(corners_in_image)

        center = self.calculate_center_of_square(position_d=position_d,
                                                 position_a=position_a,
                                                 position_b=position_b)

        if DEBUG:
            cv2.circle(image, (int(position_a[0]), int(position_a[1])), 3, (0, 0, 255), -1)
            cv2.circle(image, (int(position_b[0]), int(position_b[1])), 3, (0, 0, 255), -1)
            cv2.circle(image, (int(position_c[0]), int(position_c[1])), 3, (0, 0, 255), -1)
            cv2.circle(image, (int(position_d[0]), int(position_d[1])), 3, (0, 0, 255), -1)
            cv2.circle(image, (int(center[0]), int(center[1])), 3, (0, 0, 255), -1)
            cv2.imshow("wip", image)
            cv2.waitKey(0)

        return self.get_dictionary_of_square_position(position_a=position_a,
                                                      position_b=position_b,
                                                      position_c=position_c,
                                                      position_d=position_d,
                                                      center=center)

    def get_dictionary_of_square_position(self, position_a, position_b, position_c, position_d, center):
        return {
            "corner_A": position_a,
            "corner_B": position_b,
            "corner_C": position_c,
            "corner_D": position_d,
            "center": center
        }

    def calculate_center_of_square(self, position_d, position_a, position_b):
        center_of_square_x = int(position_a[0] - position_d[0]) / 2
        center_of_square_y = int(position_b[1] - position_a[1]) / 2
        return [position_d[0] + center_of_square_x, position_d[1] + center_of_square_y]

    def calculate_point_in_the_middle_for_each_corner(self, corners_in_image):
        position_a = self.calculate_position_a(corners_in_image["corner_A"])
        position_b = self.calculate_position_b(corners_in_image["corner_B"])
        position_c = self.calculate_position_c(corners_in_image["corner_C"])
        position_d = self.calculate_position_d(corners_in_image["corner_D"])

        return position_a, position_b, position_c, position_d

    def calculate_position_a(self, corners_of_position_a):
        if len(corners_of_position_a) != 2:
            return self.position_of_corner_a
        else:
            return self.calculate_point_in_middle(corners_of_position_a[0], corners_of_position_a[1])

    def calculate_position_b(self, param):
        if len(param) != 2:
            return self.position_of_corner_b
        else:
            return self.calculate_point_in_middle(param[0], param[1])

    def calculate_position_c(self, param):
        if len(param) != 2:
            return self.position_of_corner_c
        else:
            return self.calculate_point_in_middle(param[0], param[1])

    def calculate_position_d(self, param):
        if len(param) != 2:
            return self.position_of_corner_d
        else:
            return self.calculate_point_in_middle(param[0], param[1])

    def calculate_point_in_middle(self, p1, p2):
        return [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]
