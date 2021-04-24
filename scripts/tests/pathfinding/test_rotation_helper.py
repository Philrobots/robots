from scripts.src.pathfinding.rotation_helper import RotationHelper


class TestRotationHelper:
    """Test rotation helper"""
    @classmethod
    def setup_class(cls):
        cls.A_START_POSITION = (0, 0)
        cls.A_GRIPPER_POSITION = (1, 0)
        cls.A_CENTER_DESTINATION_POSITION = (0, 0)
        cls.A_FIRST_QUADRANT_FINAL_NODE_POSITION = (1, 0)
        cls.A_SECOND_QUADRANT_FINAL_NODE_POSITION = (0, 1)
        cls.A_THIRD_QUADRANT_FINAL_NODE_POSITION = (-1, 0)
        cls.A_FOURTH_QUADRANT_FINAL_NODE_POSITION = (0, -1)

        cls.EXPECTED_FIRST_QUADRANT_TARGET_ANGLE = 180
        cls.EXPECTED_SECOND_QUADRANT_TARGET_ANGLE = 270
        cls.EXPECTED_THIRD_QUADRANT_TARGET_ANGLE = 0
        cls.EXPECTED_FOURTH_QUADRANT_TARGET_ANGLE = 90

    def setup_method(self):
        self.rotation_helper = RotationHelper()

    def test_when_find_angle_to_turn_quadrant_1_then_return_expected_angle(self):
        target_angle = self.rotation_helper.find_angle_to_turn(
            self.A_START_POSITION,
            self.A_GRIPPER_POSITION,
            self.A_CENTER_DESTINATION_POSITION,
            self.A_FIRST_QUADRANT_FINAL_NODE_POSITION)

        assert target_angle == self.EXPECTED_FIRST_QUADRANT_TARGET_ANGLE

    def test_when_find_angle_to_turn_quadrant_2_then_return_expected_angle(self):
        target_angle = self.rotation_helper.find_angle_to_turn(
            self.A_START_POSITION,
            self.A_GRIPPER_POSITION,
            self.A_CENTER_DESTINATION_POSITION,
            self.A_SECOND_QUADRANT_FINAL_NODE_POSITION)

        assert target_angle == self.EXPECTED_SECOND_QUADRANT_TARGET_ANGLE

    def test_when_find_angle_to_turn_quadrant_3_then_return_expected_angle(self):
        target_angle = self.rotation_helper.find_angle_to_turn(
            self.A_START_POSITION,
            self.A_GRIPPER_POSITION,
            self.A_CENTER_DESTINATION_POSITION,
            self.A_THIRD_QUADRANT_FINAL_NODE_POSITION)

        assert target_angle == self.EXPECTED_THIRD_QUADRANT_TARGET_ANGLE

    def test_when_find_angle_to_turn_quadrant_4_then_return_expected_angle(self):
        target_angle = self.rotation_helper.find_angle_to_turn(
            self.A_START_POSITION,
            self.A_GRIPPER_POSITION,
            self.A_CENTER_DESTINATION_POSITION,
            self.A_FOURTH_QUADRANT_FINAL_NODE_POSITION)

        assert target_angle == self.EXPECTED_FOURTH_QUADRANT_TARGET_ANGLE
