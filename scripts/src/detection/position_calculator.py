from math import atan2, pi, hypot


class PositionCalculator:

    def calculate_angle_between_two_position(self, position_one, position_two):
        x_difference = self.calculate_difference_of_x(position_one, position_two)
        y_difference = self.calculate_difference_of_y(position_one, position_two)
        angle = atan2(y_difference, x_difference)
        return self.convert_negative_to_positive(angle)

    def calculate_difference_of_x(self, position_one, position_two):
        return position_two[0] - position_one[0]

    def calculate_difference_of_y(self, position_one, position_two):
        return -(position_two[1] - position_one[1])

    def convert_negative_to_positive(self, angle):
        if angle < 0:
            angle += 2*pi
        return angle

    def calculate_distance_between_two_points(self, point_1, point_2):
        return hypot(point_2[0] - point_1[0], point_2[1] - point_1[1])

#position_calculator = PositionCalculator()
#point_1 = (719, 490)
#point_2 = (815, 484)
#distance = position_calculator.calculate_distance_between_two_points(point_1, point_2)
