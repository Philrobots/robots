import { Coordinate } from '@/types/coordinate';
import { Step } from '@/types/step';
import { RobotConsumption } from '@/types/robotConsumption';
import { PuckList } from '@/types/puckList';
import { TableImage } from '@/types/tableImage';

export const defaultState = {
  cycleNumber: 1,
  cycleReady: false,
  cycleStarted: false,
  tableImage: {
    previous: '',
    current: '',
  } as TableImage,
  resistance: 0,
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 0,
    batteryCharge: 0,
  } as RobotConsumption,
  plannedTrajectory: [] as Array<Coordinate>,
  currentPlannedTrajectory: [] as Array<Coordinate>,
  realTrajectory: [] as Array<Coordinate>,
  currentStep: Step.CycleNotStarted as number,
  puckList: new PuckList(),
};

export const state = {
  cycleNumber: defaultState.cycleNumber,
  cycleReady: defaultState.cycleReady,
  cycleStarted: defaultState.cycleStarted,
  tableImage: defaultState.tableImage,
  resistance: defaultState.resistance,
  robotConsumption: defaultState.robotConsumption,
  plannedTrajectory: defaultState.plannedTrajectory,
  currentPlannedTrajectory: defaultState.currentPlannedTrajectory,
  realTrajectory: defaultState.realTrajectory,
  currentStep: defaultState.currentStep,
  puckList: defaultState.puckList,
};

export type State = typeof state;
