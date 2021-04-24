from enum import Enum


class Step(Enum):
    # La string correspond avec l'enum frontend/src/types/step.ts
    WAIT_FOR_ROBOT_READY_STATE = 'CycleNotStarted'
    WAIT_FOR_FRONTEND_CYCLE_START = 'CycleReadyInWaitingMode'
    MOVE_ROBOT_TO_RESISTANCE_STATION = 'ToResistanceStation'
    READ_RESISTANCE = 'ReadResistance'
    MOVE_ROBOT_TO_COMMAND_PANEL = 'ToControlPanel'
    READ_LETTERS = 'ReadControlPanel'
    GRIP_PUCK = 'GripPuck'
    RELEASE_PUCK = 'ReleasePuck'
    MOVE_ROBOT_TO_SQUARE_CENTER = 'ToSquareCenter'
    END_CYCLE = 'CycleEndedRedLedOn'
    TO_FIRST_PUCK_AND_GRAB_FIRST_PUCK = "ToFirstPuckAndGrabFirstPuck"
    TO_SECOND_PUCK_AND_GRAB_SECOND_PUCK = "ToSecondPuckAndGrabSecondPuck"
    TO_THIRD_PUCK_AND_GRAB_THIRD_PUCK = "ToThirdPuckAndGrabThirdPuck"
    TO_FIRST_CORNER_AND_RELEASE_FIRST_PUCK = "ToFirstCornerAndReleaseFirstPuck"
    TO_SECOND_CORNER_AND_RELEASE_SECOND_PUCK = "ToSecondCornerAndReleaseSecondPuck"
    TO_THIRD_CORNER_AND_RELEASE_THIRD_PUCK = "ToThirdCornerAndReleaseThirdPuck"
