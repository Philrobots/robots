import cv2

from scripts.src.detection.acuro_markers.AcuroMarkers import ArucoMarkers
from scripts.src.detection.acuro_markers.marker_position import MarkerPosition
from scripts.src.detection.acuro_markers.aruco_position import ArucoPosition


class RobotDetection(ArucoMarkers):
    def detect_aruco_marker_on_robot(self, image, DEBUG=False):
        aruco_dict = self.get_acuro_dictionnary()
        aruco_params = self.get_acuro_params()

        if image is None:
            return self.generate_empty_robot_position()

        robot_position = {}

        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict,
                                                           parameters=aruco_params)

        position = ArucoPosition(0, 0)
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                position = ArucoPosition(markerID, markerCorner)
                corners = markerCorner.reshape((4, 2))
                (top_left_position, top_right_position, bottom_right_position,
                 bottom_left_position) = corners

                bottom_left_position, bottom_right_position, top_left_position, top_right_position = \
                    self.get_markers_corners_position(
                    bottom_left_position, bottom_right_position, top_left_position,
                    top_right_position)


                center_x, center_y = self.generate_center_position(
                    bottom_right_position=bottom_right_position,
                    top_left_position=top_left_position)

                self.generate_robot_position(bottom_left_position,
                                             bottom_right_position,
                                             center_x,
                                             center_y,
                                             robot_position,
                                             top_left_position,
                                             top_right_position)

                if DEBUG:
                    self.draw_line_on_markers(bottom_left_position, bottom_right_position,
                                              image,
                                              top_left_position,
                                              top_right_position)

                    self.draw_center_position(center_x, center_y, image)

                    cv2.putText(image, str(markerID),
                            (top_left_position[0], top_left_position[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if DEBUG:
                self.show_image(image)

        return robot_position, position

    def generate_robot_position(self, bottom_left_position, bottom_right_position,
                                center_x,
                                center_y,
                                robot_position,
                                top_left_position,
                                top_right_position):
        robot_position["center"] = (center_x, center_y)
        robot_position["top_right"] = top_right_position
        robot_position["top_left"] = top_left_position
        robot_position["bottom_right"] = bottom_right_position
        robot_position["bottom_left"] = bottom_left_position

    def get_acuro_dictionnary(self):
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)

    def generate_empty_robot_position(self):
        return  {
            "center": (0, 0),
            "top_right": (0, 0),
            "top_left": (0, 0),
            "bottom_right": (0, 0),
            "bottom_left": (0, 0)
        }

    def calculate_3d_robot_position(self, robot_position: ArucoPosition,
                                    aruco_marker_width,
                                    camera_matrix,
                                    distortion_coefficient) -> MarkerPosition:

        aruco_marker_corner = robot_position.get_corner()

        rotation_vector, translation_vector, objects_point = cv2.aruco.estimatePoseSingleMarkers(
            aruco_marker_corner,
            aruco_marker_width,
            camera_matrix,
            distortion_coefficient
        )

        aruco_marker_position = MarkerPosition(
                markers_points=objects_point,
                rotation_vector=rotation_vector,
                translation_vector=translation_vector
            )

        return aruco_marker_position
