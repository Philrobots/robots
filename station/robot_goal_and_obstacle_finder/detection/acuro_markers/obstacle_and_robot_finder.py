import os
import math
from typing import List
import numpy as np
import cv2

from detection.acuro_markers.obstacle_detection import ObstacleDetection
from detection.acuro_markers.robot_detection import RobotDetection
from detection.acuro_markers.marker_position import MarkerPosition
from detection.position_calculator import PositionCalculator


class ObstacleRobotFinder:
    def __init__(self):
        self.obstacle_detection = ObstacleDetection()
        self.robot_detection = RobotDetection()
        self.position_calculator = PositionCalculator()
        #mm
        self.image_width = 1600
        self.image_height = 904
        self.obstacle_height = 412
        self.robot_height = 254
        self.aruco_marker_width = 80
        self.aruco_robot_marker_width = 143
        #pixel
        self.obstacle_radius = 42
        self.distance_between_center_and_prehenseur = 130
        self.distortion_coefficients = np.array(
            [
                [
                    0.055764032942161694,
                    -0.1700050453380352,
                    -0.0028056916670508593,
                    0.0006434607299710345,
                    0.0331770702717552
                ]
        ])
        self.camera_matrix = np.array([
            [
                1321.5030177675765,
                0.0,
                763.385168511886
            ],
            [
                0.0,
                1327.9592573621323,
                494.93250836436187
            ],
            [
                0.0,
                0.0,
                1.0
            ]
    ])

    def read_image(self, image):
        script_dir = os.path.dirname( __file__ )
        rel_path = image
        abs_file_path = os.path.join( script_dir, rel_path )
        image = cv2.imread( abs_file_path )
        return image

    def detect_obstacle_position(self, image, DEBUG=False):
        obstacles_position = self.obstacle_detection.detect_aruco_marker_on_obstacle( image )

        obstacles_3d_positions = self.obstacle_detection\
            .calculate_obstacle_position(obstacles_position=obstacles_position,
                                         aruco_marker_width=self.aruco_marker_width,
                                         camera_matrix=self.camera_matrix,
                                         distortion_coefficient=self.distortion_coefficients)

        for marker_position in obstacles_3d_positions:
            marker_position.set_markers_points(np.array([[0.0, 0.0, self.obstacle_height]]))
            marker_position.set_rotation_vector(np.array([[0.0, 0.0, 0.0]]))

        image_copy, obstacles_bottom_position = self.detect_bottom_of_obstacle(
            image=image,
            markers_position=obstacles_3d_positions)

        if DEBUG:
            cv2.putText(image_copy, "1", obstacles_bottom_position[0]["center_of_obstacle"],
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 2)

            cv2.putText(image_copy, "2", (obstacles_bottom_position[1]["center_of_obstacle"]),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 2)
            cv2.imshow("Detect obstacle", image_copy)
            cv2.waitKey(0)

        return obstacles_bottom_position

    def detect_bottom_of_obstacle(self, markers_position: List[MarkerPosition], image):
        if image is None:
            return []

        image_copy = image.copy()
        obstacles_bottom_position = []

        for marker_position in markers_position:
            center_of_bottom_obstacle, _ = cv2.projectPoints(
                marker_position.get_markers_points(),
                marker_position.get_rotation_vector(),
                marker_position.get_translation_vector(),
                self.camera_matrix,
                self.distortion_coefficients
            )

            center_of_bottom_obstacle = tuple(center_of_bottom_obstacle.reshape(2, ).astype(np.int32))
            obstacles_bottom_position.append({
                "center_of_obstacle": center_of_bottom_obstacle,
                "radius": self.obstacle_radius
            })

            image_copy = cv2.circle(image_copy, center_of_bottom_obstacle, self.obstacle_radius, (0, 255, 255), 2)

        return image_copy, obstacles_bottom_position

    def detect_robot(self, image, DEBUG=False):
        robot_position, aruco_marker_position = self.robot_detection.detect_aruco_marker_on_robot(
            image
        )

        bottom_left = robot_position["bottom_left"]
        bottom_right = robot_position["bottom_right"]

        angle_robot = self.position_calculator.calculate_angle_between_two_position(bottom_right, bottom_left)

        robot_3d_position = self.robot_detection \
            .calculate_3d_robot_position(robot_position=aruco_marker_position,
                                          aruco_marker_width=self.aruco_robot_marker_width,
                                          camera_matrix=self.camera_matrix,
                                          distortion_coefficient=self.distortion_coefficients )
        robot_3d_position.set_markers_points(np.array([[0.0, 0.0, self.robot_height]]))
        robot_3d_position.set_rotation_vector(np.array([[0.0, 0.0, 0.0]]))

        image_copy, center_of_bottom_robot, prehenseur_position = self.detect_bottom_of_robot(
            image=image,
            marker_position=robot_3d_position,
            angle=angle_robot, DEBUG=DEBUG
        )

        if DEBUG:
            cv2.putText(image, "1", bottom_left,
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(image, "2", bottom_right, cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)

            cv2.imshow("Detection robot and prehenseur", image_copy)
            cv2.waitKey(0)

        return center_of_bottom_robot, prehenseur_position, angle_robot


    def detect_bottom_of_robot(self, marker_position: MarkerPosition, image, angle, DEBUG):
        if image is None:
            return None

        image_copy = image.copy()
        center_of_bottom_of_robot, _ = cv2.projectPoints(
            marker_position.get_markers_points(),
            marker_position.get_rotation_vector(),
            marker_position.get_translation_vector(),
            self.camera_matrix,
            self.distortion_coefficients
        )

        center_of_bottom_of_robot = tuple(center_of_bottom_of_robot.reshape(2,).astype(np.int32))

        prehenseur_position = (int(center_of_bottom_of_robot[0] +
                                    (self.distance_between_center_and_prehenseur * math.cos(angle))),
                               int(center_of_bottom_of_robot[1] -
                                    (self.distance_between_center_and_prehenseur * math.sin(angle))))

        if DEBUG:
            image_copy = cv2.circle(image_copy, center_of_bottom_of_robot, 1, color=(0, 255, 255), thickness=5)
            image_copy = cv2.circle(image_copy, prehenseur_position, 1, color=(255, 255, 255), thickness=5)

            cv2.putText(image_copy, "Point central base", (center_of_bottom_of_robot[0] - 90,
                                                           center_of_bottom_of_robot[1] - 20),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 255), 2)

            cv2.putText(image_copy, "Prehenseur", (prehenseur_position[0],
                                                   prehenseur_position[1] - 20),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 2)

        return image_copy, center_of_bottom_of_robot, prehenseur_position


#AN_IMAGE = "testing7.jpg"
#image = cv2.imread(AN_IMAGE)
#obstacle_robot_finder = ObstacleRobotFinder()
#obstacle_position = obstacle_robot_finder.detect_obstacle_position(image=image, DEBUG=False)

#x, y, angle = obstacle_robot_finder.detect_robot(image, False)
