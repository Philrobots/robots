import cv2


class ArucoMarkers:

    def generate_center_position(self, bottom_right_position, top_left_position):
        center_x = int((top_left_position[0] + bottom_right_position[0]) / 2.0)
        center_y = int((top_left_position[1] + bottom_right_position[1]) / 2.0)
        return center_x, center_y

    def show_image(self, image):
        cv2.imshow("Markers", image)
        cv2.waitKey(10000)

    def draw_center_position(self, center_x, center_y, image):
        cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)

    def get_markers_corners_position(self, bottom_left_position, bottom_right_position,
                                     top_left_position,
                                     top_right_position):
        top_right_position = (int(top_right_position[0]), int(top_right_position[1]))
        bottom_right_position = (int(bottom_right_position[0]), int(bottom_right_position[1]))
        bottom_left_position = (int(bottom_left_position[0]), int(bottom_left_position[1]))
        top_left_position = (int(top_left_position[0]), int(top_left_position[1]))
        return bottom_left_position, bottom_right_position, top_left_position, top_right_position


    def draw_line_on_markers(self, bottom_left_position, bottom_right_position, image,
                             top_left_position,
                             top_right_position):
        cv2.line(image, top_left_position, top_right_position, (0, 255, 0), 2)
        cv2.line(image, top_right_position, bottom_right_position, (0, 255, 0), 2)
        cv2.line(image, bottom_right_position, bottom_left_position, (0, 255, 0), 2)
        cv2.line(image, bottom_left_position, top_left_position, (0, 255, 0), 2)

    def get_acuro_params(self):
        return cv2.aruco.DetectorParameters_create()

    def detect_markers(self, aruco_dict, aruco_params, image_with_obstacle):
        return cv2.aruco.detectMarkers(image_with_obstacle, aruco_dict,
                                       parameters=aruco_params)

    def generate_obstacle_dict(self, top_right, top_left, bottom_right, bottom_left,
                               obstacle_id, center_x, center_y):
        obstacle_report = {
            f"obstacle {obstacle_id}": {
            "center": (center_x, center_y),
            "top_right": top_right,
            "top_left": top_left,
            "bottom_right": bottom_right,
            "bottom_left": bottom_left
            }
        }
        return obstacle_report
