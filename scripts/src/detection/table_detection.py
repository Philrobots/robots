import cv2


from scripts.src.detection.object_detection import ObjectDetection
from scripts.src.detection.position import Position


class TableDetection(ObjectDetection):
    def __init__(self, image):
        super().__init__(image, "table", 760, 1570)
        self.minimum_area = 1150000
        self.maximum_area = 1250000

    def detect_table(self):
        cv2.namedWindow('Table detection', cv2.WINDOW_NORMAL)
        image_copy = self.copy_image()
        table_position = self._find_color(image_copy)
        self._show_image(image_copy)
        return table_position

    def _find_color(self, image_copy):
        image_hsv = cv2.cvtColor( self.image, cv2.COLOR_BGR2HSV )

        color_lower_boundary = self.lower_boundary.get_lower_boundaries(self.object_to_detect)
        color_upper_boundary = self.upper_boundary.get_upper_boundaries(self.object_to_detect)

        mask = cv2.inRange(image_hsv, color_lower_boundary, color_upper_boundary)
        return self._get_contours(mask, image_copy)

    def _get_contours(self, image_mask, image_copy):
        contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corner_position = {}

        for contour in contours:
            area = cv2.contourArea(contour)
            if self.is_in_area(area):
                perimeter = cv2.arcLength(contour, True)
                zone_approximation = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
                object_corner = len( zone_approximation )
                x_position, y_position, width, height = cv2.boundingRect(zone_approximation)
                self.top_left_corner_x = x_position
                self.top_left_corner_y = y_position
                self.width = width
                self.height = height

                if self.object_is_in_range(width, height):
                    self.draw_rectangle_on_image( image_copy, x_position, y_position, width, height,
                                                  self.get_object_name( object_corner ) )

                    corner_position = self.generate_four_corners(x_position, y_position,
                                                                 width, height)

        return corner_position


    def generate_four_corners(self, x_position, y_position, width, height):
        corner_a = Position(x_position, y_position)
        corner_b = Position(x_position + width, y_position)
        corner_c = Position(x_position + width, y_position + height)
        corner_d = Position(x_position, y_position + height)

        return {
            "corner_A": corner_a,
            "corner_B": corner_b,
            "corner_C": corner_c,
            "corner_D": corner_d
        }

    def get_object_name(self, object_corner):
        if object_corner == 4:
            object_type = str(self.object_to_detect)
        else:
            object_type = "None"
        return object_type

    def is_point_in_table(self, position, top_left_corner, width, height):
        top_left_corner_x = top_left_corner.get_position_x()
        top_left_corner_y = top_left_corner.get_position_y()
        if top_left_corner_x < position.get_position_x() < top_left_corner_x + width and \
                top_left_corner_y < position.get_position_y() < top_left_corner_y + height:
            return True
        return False


    def is_in_area(self, area):
        return self.minimum_area < area < self.maximum_area
