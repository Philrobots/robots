import GripState from '@/components/objectives/GripState.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';
import { State } from '@/store/state';
import { PuckListFactory } from '@/factories/PuckListFactory';
import { PuckState } from '@/types/puckState';

const mockState = (puckStates: Array<PuckState>): State => {
  return {
    puckList: PuckListFactory.makeWithStates(puckStates),
  } as State;
};

describe('When mounting GripState component', () => {
  const wrapper = wrapComponentForTest(GripState);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given puck in grip', () => {
  const state = mockState([
    PuckState.GRIPPED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]);

  describe('When mounting GripState', () => {
    const wrapper = wrapComponentForTest(GripState, state);

    it('Should contain the right value', () => {
      const grip = wrapper.findComponent({ ref: 'gripState' });

      expect(grip.exists()).toBe(true);
      expect(grip.text()).toBe(wrapper.vm.$t('objectives.puckInGrip'));
    });
  });
});

describe('Given puck released', () => {
  const state = mockState([
    PuckState.RELEASED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]);

  describe('When mounting GripState', () => {
    const wrapper = wrapComponentForTest(GripState, state);

    it('Should contain the right value', () => {
      const grip = wrapper.findComponent({ ref: 'gripState' });

      expect(grip.exists()).toBe(true);
      expect(grip.text()).toBe(wrapper.vm.$t('objectives.noPuck'));
    });
  });
});
