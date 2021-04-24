from rospy.exceptions import ROSInitException

from commands.step import Step
from commands.command_builder import CommandBuilder

from handlers.wait_for_robot_ready_state_handler import WaitForRobotReadyStateHandler
from handlers.wait_for_frontend_cycle_start_handler import WaitForFrontendCycleStartHandler
from handlers.move_robot.move_robot_to_resistance_station_handler import MoveRobotToResistanceStationHandler
from handlers.move_robot.move_robot_to_command_panel_handler import MoveRobotToCommandPanelHandler
from handlers.move_robot.move_robot_to_next_puck_handler import MoveRobotToNextPuckHandler
from handlers.move_robot.move_robot_to_next_corner_handler import MoveRobotToNextCornerHandler
from handlers.move_robot.move_robot_to_square_center_handler import MoveRobotToSquareCenterHandler
from handlers.read_resistance_handler import ReadResistanceHandler
from handlers.read_letters_handler import ReadLettersHandler
from handlers.grip_puck_handler import GripPuckHandler
from handlers.release_puck_handler import ReleasePuckHandler
from handlers.end_cycle_handler import EndCycleHandler

command_builder = CommandBuilder()


def test_given_no_step_when_building_then_return_empty_list():
    steps = []

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == 0


def test_given_multiple_steps_when_building_then_list_of_length_of_steps():
    steps = [Step.MOVE_ROBOT_TO_RESISTANCE_STATION, Step.MOVE_ROBOT_TO_RESISTANCE_STATION]

    commands = command_builder.with_steps(steps).build_many()

    assert len(commands) == len(steps)


def test_given_wait_for_robot_ready_state_step_when_building_then_return_associated_command():
    step = Step.WAIT_FOR_ROBOT_READY_STATE
    handler_classes = [WaitForRobotReadyStateHandler]

    try:
        given_single_step_when_building_then_return_associated_command(step, handler_classes)
    except ROSInitException:
        pass


def test_given_wait_for_frontend_cycle_start_step_when_building_then_return_associated_command():
    step = Step.WAIT_FOR_FRONTEND_CYCLE_START
    handler_classes = [WaitForFrontendCycleStartHandler]

    try:
        given_single_step_when_building_then_return_associated_command(step, handler_classes)
    except ROSInitException:
        pass


def test_given_move_robot_to_resistance_station_step_when_building_then_return_associated_command():
    step = Step.MOVE_ROBOT_TO_RESISTANCE_STATION
    handler_classes = [MoveRobotToResistanceStationHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_read_resistance_step_when_building_then_return_associated_command():
    step = Step.READ_RESISTANCE
    handler_classes = [ReadResistanceHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_move_robot_to_command_panel_step_when_building_then_return_associated_command():
    step = Step.MOVE_ROBOT_TO_COMMAND_PANEL
    handler_classes = [MoveRobotToCommandPanelHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_read_letters_step_when_building_then_return_associated_command():
    step = Step.READ_LETTERS
    handler_classes = [ReadLettersHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_move_robot_to_next_puck_step_when_building_then_return_associated_command():
    step = Step.TO_FIRST_PUCK_AND_GRAB_FIRST_PUCK
    handler_classes = [MoveRobotToNextPuckHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_grip_puck_step_when_building_then_return_associated_command():
    step = Step.GRIP_PUCK
    handler_classes = [GripPuckHandler]

    try:
        given_single_step_when_building_then_return_associated_command(step, handler_classes)
    except ROSInitException:
        pass


def test_given_move_robot_to_next_corner_step_when_building_then_return_associated_command():
    step = Step.TO_FIRST_CORNER_AND_RELEASE_FIRST_PUCK
    handler_classes = [MoveRobotToNextCornerHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_release_puck_step_when_building_then_return_associated_command():
    step = Step.RELEASE_PUCK
    handler_classes = [ReleasePuckHandler]

    try:
        given_single_step_when_building_then_return_associated_command(step, handler_classes)
    except ROSInitException:
        pass


def test_given_move_robot_to_square_center_step_when_building_then_return_associated_command():
    step = Step.MOVE_ROBOT_TO_SQUARE_CENTER
    handler_classes = [MoveRobotToSquareCenterHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def test_given_end_cycle_step_when_building_then_return_associated_command():
    step = Step.END_CYCLE
    handler_classes = [EndCycleHandler]

    given_single_step_when_building_then_return_associated_command(step, handler_classes)


def given_single_step_when_building_then_return_associated_command(step, handler_classes):
    commands = command_builder.some_commands().with_steps([step]).build_many()

    assert len(commands) == 1
    assert len(commands[0].handlers) == len(handler_classes)

    for idx, handler_class in enumerate(handler_classes):
        assert isinstance(commands[0].handlers[idx], handler_class)
