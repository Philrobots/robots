from handlers.end_cycle_handler import EndCycleHandler
from handlers.grip_puck_handler import GripPuckHandler
from handlers.move_robot.move_robot_to_command_panel_handler import \
    MoveRobotToCommandPanelHandler
from handlers.move_robot.move_robot_to_next_corner_handler import \
    MoveRobotToNextCornerHandler
from handlers.move_robot.move_robot_to_next_puck_handler import \
    MoveRobotToNextPuckHandler
from handlers.move_robot.move_robot_to_resistance_station_handler import \
    MoveRobotToResistanceStationHandler
from handlers.move_robot.move_robot_to_square_center_handler import \
    MoveRobotToSquareCenterHandler
from handlers.read_letters_handler import ReadLettersHandler
from handlers.read_resistance_handler import ReadResistanceHandler
from handlers.release_puck_handler import ReleasePuckHandler
from handlers.wait_for_frontend_cycle_start_handler import \
    WaitForFrontendCycleStartHandler
from handlers.wait_for_robot_ready_state_handler import \
    WaitForRobotReadyStateHandler

from commands.command import Command
from commands.step import Step


class CommandBuilder:
    _commands = []

    def some_commands(self):
        self._commands = []
        return self

    def with_steps(self, steps):
        for step in steps:
            self._with_step(step)

        return self

    def _with_step(self, step):
        if step == Step.WAIT_FOR_ROBOT_READY_STATE:
            self._commands.append(Command([WaitForRobotReadyStateHandler()], step))
        elif step == Step.WAIT_FOR_FRONTEND_CYCLE_START:
            self._commands.append(Command([WaitForFrontendCycleStartHandler()], step))
        elif step == Step.MOVE_ROBOT_TO_RESISTANCE_STATION:
            self._commands.append(Command([MoveRobotToResistanceStationHandler()], step))
        elif step == Step.READ_RESISTANCE:
            self._commands.append(Command([ReadResistanceHandler()], step))
        elif step == Step.MOVE_ROBOT_TO_COMMAND_PANEL:
            self._commands.append(Command([MoveRobotToCommandPanelHandler()], step))
        elif step == Step.READ_LETTERS:
            self._commands.append(Command([ReadLettersHandler()], step))
        elif step in (Step.TO_FIRST_PUCK_AND_GRAB_FIRST_PUCK, Step.TO_SECOND_PUCK_AND_GRAB_SECOND_PUCK, Step.TO_THIRD_PUCK_AND_GRAB_THIRD_PUCK):
            self._commands.append(Command([MoveRobotToNextPuckHandler()], step))
        elif step == Step.GRIP_PUCK:
            self._commands.append(Command([GripPuckHandler()]))
        elif step in (Step.TO_FIRST_CORNER_AND_RELEASE_FIRST_PUCK, Step.TO_SECOND_CORNER_AND_RELEASE_SECOND_PUCK, Step.TO_THIRD_CORNER_AND_RELEASE_THIRD_PUCK):
            self._commands.append(Command([MoveRobotToNextCornerHandler()], step))
        elif step == Step.RELEASE_PUCK:
            self._commands.append(Command([ReleasePuckHandler()]))
        elif step == Step.MOVE_ROBOT_TO_SQUARE_CENTER:
            self._commands.append(Command([MoveRobotToSquareCenterHandler()], step))
        elif step == Step.END_CYCLE:
            self._commands.append(Command([EndCycleHandler()], step))

    def build_many(self):
        return self._commands
