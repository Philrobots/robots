import { MutationTree } from 'vuex/types';
import {
  RESET_CYCLE,
  SOCKET_CURRENT_STEP,
  SOCKET_GRIP_STATE,
  SOCKET_PLANNED_TRAJECTORY_COORDINATES,
  SOCKET_PUCK_COLORS,
  SOCKET_PUCK_FIRST_CORNER,
  SOCKET_REAL_TRAJECTORY_COORDINATE,
  SOCKET_RESISTANCE,
  SOCKET_ROBOT_CONSUMPTION,
  SOCKET_TABLE_IMAGE,
  START_CYCLE,
} from './mutation-types';
import { defaultState, State } from './state';
import { Message } from '@/types/message';
import { Step } from '@/types/step';

export type Mutations<S = State> = {
  [START_CYCLE](state: S): void;
  [RESET_CYCLE](state: S): void;
  [SOCKET_ROBOT_CONSUMPTION](state: S, data: string): void;
  [SOCKET_RESISTANCE](state: S, data: string): void;
  [SOCKET_PUCK_COLORS](state: S, data: string): void;
  [SOCKET_PUCK_FIRST_CORNER](state: S, data: string): void;
  [SOCKET_PLANNED_TRAJECTORY_COORDINATES](state: S, data: string): void;
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: S, data: string): void;
  [SOCKET_GRIP_STATE](state: S, data: string): void;
  [SOCKET_CURRENT_STEP](state: S, data: string): void;
};

const toMessage = (data: string): Message => JSON.parse(data);

export const mutations: MutationTree<State> & Mutations = {
  [START_CYCLE](state: State) {
    state.currentStep = Step.CycleStarted;
    state.cycleStarted = true;
  },
  [RESET_CYCLE](state: State) {
    const cycleNumber = state.cycleNumber + 1;
    Object.assign(state, { ...defaultState, cycleNumber });
  },
  [SOCKET_ROBOT_CONSUMPTION](state: State, data: string) {
    const message = toMessage(data);
    state.cycleReady = true;
    state.robotConsumption =
      message.robotConsumption || defaultState.robotConsumption;
  },
  [SOCKET_TABLE_IMAGE](state: State, data: string) {
    const message = toMessage(data);
    state.tableImage.previous =
      state.tableImage.current || defaultState.tableImage.previous;
    state.tableImage.current =
      message.tableImage ||
      state.tableImage.previous ||
      defaultState.tableImage.current;
  },
  [SOCKET_RESISTANCE](state: State, data: string) {
    const message = toMessage(data);
    state.resistance = message.resistance || defaultState.resistance;
  },
  [SOCKET_PUCK_COLORS](state: State, data: string) {
    const message = toMessage(data);
    state.puckList.colors = message.puckColors || defaultState.puckList.colors;
  },
  [SOCKET_PUCK_FIRST_CORNER](state: State, data: string) {
    const message = toMessage(data);
    state.puckList.firstCorner =
      message.puckFirstCorner || defaultState.puckList.first.corner;
  },
  [SOCKET_PLANNED_TRAJECTORY_COORDINATES](state: State, data: string) {
    const message = toMessage(data);
    if (message.plannedTrajectoryCoordinates) {
      state.plannedTrajectory.push(...message.plannedTrajectoryCoordinates);
      state.currentPlannedTrajectory =
        message.plannedTrajectoryCoordinates ||
        defaultState.currentPlannedTrajectory;
    }
  },
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: State, data: string) {
    const message = toMessage(data);
    if (message.realTrajectoryCoordinate)
      state.realTrajectory.push(message.realTrajectoryCoordinate);
  },
  [SOCKET_GRIP_STATE](state: State, data: string) {
    const message = toMessage(data);
    state.puckList.hasOneGripped = message.puckInGrip || false;
  },
  [SOCKET_CURRENT_STEP](state: State, data: string) {
    const message = toMessage(data);
    state.currentStep = message.currentStep
      ? Step[message.currentStep as keyof typeof Step]
      : defaultState.currentStep;
  },
};
