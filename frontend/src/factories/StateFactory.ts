import { factory } from 'node-factory';
import { defaultState, State } from '@/store/state';
import { CoordinateFactory } from '@/factories/CoordinateFactory';
import { Coordinate } from '@/types/coordinate';
import { PuckListFactory } from '@/factories/PuckListFactory';

const TRAJECTORY_POINTS = 20;
const CURRENT_TRAJECTORY_POINTS = TRAJECTORY_POINTS / 2;

export const StateFactory = factory<State>((fake) => {
  const plannedTrajectory = CoordinateFactory.make(TRAJECTORY_POINTS);

  const fakeRealPoints = (coordinates: Array<Coordinate>) => {
    return coordinates.map((coordinate) => {
      const fakeFactor =
        (fake.random.boolean() ? -1 : 1) * fake.random.number(20);
      return {
        x: coordinate.x + fakeFactor,
        y: coordinate.y + fakeFactor,
      } as Coordinate;
    });
  };

  return {
    cycleNumber: fake.random.number(10),
    cycleReady: defaultState.cycleReady,
    cycleStarted: defaultState.cycleStarted,
    tableImage: {
      current: '/stub_table_image.jpg',
      previous: '',
    },
    resistance: fake.random.number(10000),
    robotConsumption: defaultState.robotConsumption,
    plannedTrajectory,
    currentPlannedTrajectory: plannedTrajectory.slice(
      0,
      CURRENT_TRAJECTORY_POINTS
    ),
    realTrajectory: fakeRealPoints(plannedTrajectory),
    currentStep: defaultState.currentStep,
    puckList: PuckListFactory.make(),
  };
});
