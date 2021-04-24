from typing import List

import cv2

from scripts.src.detection.acuro_markers.AcuroMarkers import ArucoMarkers
from scripts.src.detection.acuro_markers.aruco_position import ArucoPosition
from scripts.src.detection.acuro_markers.marker_position import MarkerPosition


class ObstacleDetection(ArucoMarkers):

    def detect_aruco_marker_on_obstacle(self, image):
        aruco_dict = self.get_acuro_dictionnary()
        aruco_params = self.get_acuro_params()

        if image is None:
            return self.generate_empty_obstacle_position()

        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict,
                                       parameters=aruco_params)

        obstacles_position = []

        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                obstacles_position.append( ArucoPosition( markerID, markerCorner ) )
        return obstacles_position

    def get_acuro_dictionnary(self):
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    def generate_empty_obstacle_position(self):
        obstacle_position = []
        for i in range(0, 2):
            obstacle_position.append({f"obstacle {i + 1}": {
                "center": (0, 0),
                "top_right": (0, 0),
                "top_left": (0, 0),
                "bottom_right": (0, 0),
                "bottom_left": (0, 0)
            }})
        return obstacle_position

    def calculate_obstacle_position(self, obstacles_position: List[ArucoPosition],
                                    aruco_marker_width,
                                    camera_matrix,
                                    distortion_coefficient) -> List[MarkerPosition]:

        aruco_markers_corner = [obstacle_position.get_corner()
                                for obstacle_position in
                                obstacles_position]

        corner_length = len(aruco_markers_corner)

        if corner_length < 1:
            return []

        rotation_vectors, translation_vectors, objects_points = cv2.aruco.estimatePoseSingleMarkers(
            aruco_markers_corner,
            aruco_marker_width,
            camera_matrix,
            distortion_coefficient
        )

        aruco_markers_positions = [
            MarkerPosition(
                markers_points=object_point,
                rotation_vector=rotation_vector,
                translation_vector=translation_vector
            )
            for rotation_vector, translation_vector, object_point
            in zip(rotation_vectors, translation_vectors, objects_points)
        ]

        return aruco_markers_positions
