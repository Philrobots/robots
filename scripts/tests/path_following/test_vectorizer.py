import math

from scripts.src.path_following.destination import Destination
from scripts.src.path_following.vectorizer import Vectorizer
from scripts.src.path_following.movement_mode import MovementMode


class TestVectorizer:
    def setup_method(self):
        self.vectorizer = Vectorizer()

    def test_given_perpendicular_nodes_when_vectorize_then_return_correct_vectors(self):
        nodes = [
            (0, 0), (1, 0), (1, -1), (0, -1), (0, 0)
        ]

        vectors = self.vectorizer.vectorize(nodes)

        assert vectors == [[1, 0], [1, math.pi/2], [1, math.pi], [1, -math.pi/2]]

    def test_given_diagonal_nodes_when_vectorize_then_return_correct_vectors(self):
        nodes = [
            (-1, -1), (0, 0), (1, -1), (0, 0), (-1, -1)
        ]

        vectors = self.vectorizer.vectorize(nodes)

        assert vectors == [[math.sqrt(2), -math.pi/4], [math.sqrt(2), math.pi/4], [math.sqrt(2), -math.pi*(3/4)], [math.sqrt(2), math.pi*(3/4)]]

    def test_given_nodes_with_magnitude_bigger_than_1_when_vectorize_then_return_correct_vectors(self):
        nodes = [
            (0, 0), (3, 4)
        ]

        vectors = self.vectorizer.vectorize(nodes)

        vector = vectors[0]
        assert vector[0] == 5

    def test_given_diagonal_vectors_1_when_adjust_first_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, math.pi*(3/4))
        vector = (1, -math.pi/2)

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, math.pi*(3/4)]

    def test_given_diagonal_vectors_2_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, math.pi*(1/4))
        vector = (1, -math.pi*(1/4))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, -math.pi*(1/2)]

    def test_given_two_negative_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, -math.pi/2)
        vector = (1, -math.pi*(3/4))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, -math.pi/4]

    def test_given_two_positive_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, 0)
        vector = (1, math.pi/2)

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, math.pi/2]

    def test_given_two_equal_vectors_when_adjust_vector_angle_from_robot_pov_then_return_correct_angle(self):
        last_vector = (None, math.pi)
        vector = (1, math.pi)

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, 0]

    def test_given_two_positive_vectors_where_difference_is_bigger_than_pi_when_adjust_vector_angle_from_robot_pov_then_correction_angle_is_negative(self):
        last_vector = (None, 0)
        vector = (1, math.pi*(3/2))

        corrected_vector = self.vectorizer.adjust_vector_angle_from_robot_pov(last_vector, vector)

        assert corrected_vector == [1, -math.pi/2]

    def test_given_everything_aligned_when_adjust_vector_angles_from_robot_pov_then_return_correct_vectors(self):
        robot_angle = 0
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, 0), (1, 0), (1, 0)
        ]

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        assert adjusted_angles == [0, 0, 0]

    def test_given_everything_aligned_but_robot_when_adjust_vector_angles_from_robot_pov_then_return_correct_vectors(self):
        robot_angle = math.pi/2
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, 0), (1, 0), (1, 0)
        ]

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        assert adjusted_angles == [-math.pi/2, 0, 0]

    def test_given_complex_alignment_when_adjust_vector_angles_from_robot_pov_then_return_correct_vectors(self):
        robot_angle = math.pi*(3/4)
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, -math.pi/4), (1, -math.pi/2), (1, -math.pi/2), (1, math.pi), (1, math.pi),
            (1, -math.pi*(3/4)), (1, -math.pi*(3/4))
        ]

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        assert adjusted_angles == [math.pi, -math.pi/4, 0, -math.pi/2, 0, math.pi/4, 0]

    def test_when_minimize_vectors_then_return_correct_vectors(self):
        robot_angle = math.pi*(3/4)
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, -math.pi / 4), (1, -math.pi / 2), (1, -math.pi / 2), (1, math.pi), (1, math.pi),
            (1, -math.pi * (3 / 4)), (1, -math.pi * (3 / 4))
        ]
        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        minimized_vectors = self.vectorizer.minimize_vectors(adjusted_vectors)
        print(minimized_vectors)
        assert minimized_vectors == [
            [1, math.pi, MovementMode.GRIP], [2, -math.pi/4, MovementMode.GRIP], [2, -math.pi/2, MovementMode.GRIP], [2, math.pi/4, MovementMode.GRIP]
        ]

    def test_given_first_vector_is_aligned_when_minimize_vectors_then_return_correct_vectors(self):
        robot_angle = -math.pi/4
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, -math.pi / 4), (1, -math.pi / 2), (1, -math.pi / 2), (1, math.pi), (1, math.pi),
            (1, -math.pi * (3 / 4)), (1, -math.pi * (3 / 4))
        ]
        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        minimized_vectors = self.vectorizer.minimize_vectors(adjusted_vectors)

        assert minimized_vectors == [
            [1, 0, MovementMode.GRIP], [2, -math.pi/4, MovementMode.GRIP], [2, -math.pi/2, MovementMode.GRIP], [2, math.pi/4, MovementMode.GRIP]
        ]

    def test_given_checkpoint_is_none_when_get_path_from_robot_then_give_path_from_start(self):
        robot_position = (-100, -100)
        self.vectorizer.set_robot_position(robot_position)
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.checkpoint = None

        corrected_path = self.vectorizer.get_path_from_robot(nodes)

        assert corrected_path == [(-100, -100), (0, 0), (15, 0), (30, 0)]

    def test_given_robot_is_close_to_path_and_was_updated_recently_when_get_path_from_robot_then_give_path_to_next_checkpoint(self):
        robot_position = (5, 0)
        self.vectorizer.set_robot_position(robot_position)
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_path(nodes)
        self.vectorizer.checkpoint = 0

        corrected_path = self.vectorizer.get_path_from_robot(nodes)

        assert corrected_path == [(5, 0), (15, 0), (30, 0)]

    def test_when_robot_is_close_to_path_but_wasnt_updated_recently_when_get_path_from_robot_then_give_path_to_closest_non_visited_checkpoint(self):
        robot_position = (5, 0)
        self.vectorizer.set_robot_position(robot_position)
        nodes = [
            (-100, -100), (15, 0), (30, 0)
        ]
        self.vectorizer.set_path(nodes)
        self.vectorizer.checkpoint = 0

        corrected_path = self.vectorizer.get_path_from_robot(nodes)

        assert corrected_path == [(5, 0), (15, 0), (30, 0)]

    def test_given_robot_is_close_to_path_but_wasnt_updated_recently_when_get_path_from_robot_then_give_path_to_closest_non_visited_checkpoint_2(self):
        robot_position = (25, 0)
        self.vectorizer.set_robot_position(robot_position)
        nodes = [
            (-100, -100), (15, 0), (30, 0)
        ]
        self.vectorizer.set_path(nodes)
        self.vectorizer.checkpoint = 0

        corrected_path = self.vectorizer.get_path_from_robot(nodes)

        assert corrected_path == [(25, 0), (30, 0)]

    def test_given_robot_is_not_close_to_path_when_get_path_from_robot_then_give_path_to_closest_non_visited_checkpoint(self):
        robot_position = (70, 70)
        self.vectorizer.set_robot_position(robot_position)
        nodes = [
            (-100, -100), (15, 0), (30, 0)
        ]
        self.vectorizer.set_path(nodes)
        self.vectorizer.checkpoint = 0

        corrected_path = self.vectorizer.get_path_from_robot(nodes)

        assert corrected_path == [(70, 70), (30, 0)]

    def test_given_robot_is_close_to_a_checkpoint_and_last_checkpoint_is_further_when_set_robot_position_then_dont_update_checkpoint(self):
        robot_position = (3, 0)
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_path(nodes)
        self.vectorizer.checkpoint = 1

        self.vectorizer.set_robot_position(robot_position)

        assert self.vectorizer.checkpoint == 1

    def test_given_robot_is_close_to_a_checkpoint_and_last_checkpoint_is_past_when_set_robot_position_then_update_checkpoint(self):
        robot_position = (10, 0)
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_path(nodes)
        self.vectorizer.checkpoint = 0

        self.vectorizer.set_robot_position(robot_position)

        assert self.vectorizer.checkpoint == 1

    def test_given_robot_is_far_from_all_checkpoints_when_set_goal_then_dont_update(self):
        robot_position = (100, 100)
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_path(nodes)

        self.vectorizer.set_robot_position(robot_position)

        assert self.vectorizer.checkpoint is None

    def test_when_set_path_then_checkpoint_is_none(self):
        self.vectorizer.checkpoint = 0

        self.vectorizer.set_path([])

        assert self.vectorizer.checkpoint is None

    def test_given_robot_is_at_last_checkpoint_when_get_path_from_from_robot_then_return_robot(self):
        robot_position = [30, 0]
        nodes = [
            (-100, -100), (15, 0), (30, 0)
        ]
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position(robot_position)

        corrected_path = self.vectorizer.get_path_from_robot(nodes)

        assert corrected_path == [[30, 0]]

    def test_given_path_has_one_node_and_robot_is_near_node_and_destination_other_when_path_to_vectors_then_return_correction_angle(self):
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_goal((30, 30))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((30, 0))
        self.vectorizer.set_robot_angle(0)

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == []

    def test_given_path_has_one_node_and_robot_is_near_node_and_angle_is_good_when_path_to_vectors_then_return_empty_vectors(self):
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_goal((30, 30))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((30, 0))
        self.vectorizer.set_robot_angle(-math.pi/2)

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == []

    def test_given_robot_is_not_on_correct_path_and_off_final_angle_from_goal_when_path_to_vectors_then_return_two_vectors(self):
        nodes = [
            (30, 0)
        ]
        self.vectorizer.set_goal((30, 30))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((0, 0))
        self.vectorizer.set_robot_angle(0)

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == [[30, 0, MovementMode.GRIP]]

    def test_given_robot_is_on_last_node_and_last_node_is_goal_when_path_to_vectors_then_return_empty_vectors(self):
        nodes = [
            (30, 0)
        ]
        self.vectorizer.set_goal((30, 0))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((30, 0))
        self.vectorizer.set_robot_angle(math.pi)

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == []

    def test_given_robot_is_on_last_node_and_last_node_is_goal_2_when_path_to_vectors_then_return_empty_vectors(self):
        nodes = [
            (30, 0)
        ]
        self.vectorizer.set_goal((30, 0))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((30, 0))
        self.vectorizer.set_robot_angle(0)

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == []

    def test_given_minimize_is_true_when_path_to_vectors_then_vectors_are_minimal(self):
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_goal((30, 30))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((0, 0))
        self.vectorizer.set_robot_angle(0)
        self.vectorizer.minimize = True

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == [[30, 0, MovementMode.GRIP]]

    def test_given_ohmmeter_mode_when_path_to_vectors_then_ohmmeter_mode_is_on_and_angles_are_adjusted(self):
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_destination(Destination.RESISTANCE_STATION)
        self.vectorizer.set_goal((30, 30))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((0, 0))
        self.vectorizer.set_robot_angle(0)
        self.vectorizer.set_mode(MovementMode.OHMMETER)

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == [[15, math.pi/2, MovementMode.OHMMETER], [15, 0, MovementMode.OHMMETER], [0, -math.pi/2, MovementMode.OHMMETER]]

    def test_given_mode_ohmmeter_when_adjust_vectors_angle_from_robot_pov_then_angles_are_adjusted(self):
        robot_angle = math.pi*(3/4)
        self.vectorizer.set_robot_angle(robot_angle)
        vectors = [
            (1, -math.pi/4), (1, -math.pi/2), (1, -math.pi/2), (1, math.pi), (1, math.pi),
            (1, -math.pi*(3/4)), (1, -math.pi*(3/4))
        ]
        self.vectorizer.set_mode(MovementMode.OHMMETER)

        adjusted_vectors = self.vectorizer.adjust_vector_angles_from_robot_pov(vectors)

        adjusted_angles = [vector[1] for vector in adjusted_vectors]
        assert adjusted_angles == [-math.pi/2, -math.pi/4, 0, -math.pi/2, 0, math.pi/4, 0]

    def test_given_destination_puck_mode_when_set_path_then_shorten_path_accordingly(self):
        goal = [200, 0]
        nodes = [
            [0, 0], [50, 0], [75, 0], [90, 0], [100, 0], [200, 0]
        ]
        self.vectorizer.set_goal(goal)
        self.vectorizer.set_destination(Destination.PUCK)

        self.vectorizer.set_path(nodes)

        assert self.vectorizer.path == [[0, 0], [50, 0], [75, 0], [90, 0]]

    def test_given_destination_puck_when_path_to_vectors_then_the_last_vector_is_an_angle_correction(self):
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_destination(Destination.PUCK)
        self.vectorizer.set_goal((30, 140))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((0, 0))
        self.vectorizer.set_robot_angle(0)
        self.vectorizer.set_mode(MovementMode.OHMMETER)

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == [[15, math.pi / 2, MovementMode.OHMMETER], [15, 0, MovementMode.OHMMETER],
                           [0, -math.pi / 2, MovementMode.OHMMETER]]

    def test_given_destination_resistance_station_when_path_to_vectors_then_the_last_vector_is_an_angle_correction(self):
        nodes = [
            (0, 0), (15, 0), (30, 0)
        ]
        self.vectorizer.set_destination(Destination.RESISTANCE_STATION)
        self.vectorizer.set_goal((30, 0))
        self.vectorizer.set_path(nodes)
        self.vectorizer.set_robot_position((0, 0))
        self.vectorizer.set_robot_angle(0)
        self.vectorizer.set_mode(MovementMode.GRIP)

        vectors = self.vectorizer.path_to_vectors()

        assert vectors == [[15, 0, MovementMode.GRIP], [15, 0, MovementMode.GRIP],
                           [0, -math.pi / 2, MovementMode.GRIP]]
