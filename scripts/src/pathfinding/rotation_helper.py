"""Helper module about different rotation angle related tasks"""

import math


class RotationHelper:
    """
    Helper class used to find the angle the robot needs to turn
    to be aligned with the puck before picking it up
    """
    @staticmethod
    def find_angle_to_turn(start, gripper, end, final_path_node):
        """finds the angle the robot needs to turn
        to be aligned with the puck before picking it up"""
        first_point_x, first_point_y = start
        second_point_x, second_point_y = gripper
        initial_angle = math.atan2(second_point_y - first_point_y, second_point_x - first_point_x)

        first_point_x, first_point_y = end
        second_point_x, second_point_y = final_path_node
        final_angle = math.atan2(second_point_y - first_point_y, second_point_x - first_point_x)

        target = final_angle + math.pi
        rotation = target - initial_angle

        rotation = rotation % (2 * math.pi)
        return rotation * 180 / math.pi
