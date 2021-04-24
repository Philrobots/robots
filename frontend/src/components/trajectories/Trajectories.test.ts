import Trajectories from '@/components/trajectories/Trajectories.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';
import { State } from '@/store/state';
import { CoordinateFactory } from '@/factories/CoordinateFactory';

describe('When mounting Trajectories', () => {
  const state = {
    tableImage: {
      previous: '/stub_table_image.jpg',
      current: '/stub_table_image.jpg',
    },
    plannedTrajectory: CoordinateFactory.make(4),
    currentPlannedTrajectory: CoordinateFactory.make(4),
    realTrajectory: CoordinateFactory.make(4),
  } as State;

  const wrapper = wrapComponentForTest(Trajectories, state);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
