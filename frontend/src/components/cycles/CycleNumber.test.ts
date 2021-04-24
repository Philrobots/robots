import wrapComponentForTest from '../../util/wrapComponentForTest';
import { State } from '@/store/state';
import CycleNumber from '@/components/cycles/CycleNumber.vue';

describe('When mounting CycleNumber component', () => {
  const wrapper = wrapComponentForTest(CycleNumber);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given cycle number', () => {
  const state = {
    cycleNumber: 88,
  } as State;

  describe('When mounting CycleNumber', () => {
    const wrapper = wrapComponentForTest(CycleNumber, state);

    it('Should display cycle number', () => {
      const cycleNumber = wrapper.findComponent({ ref: 'cycleNumber' });

      expect(cycleNumber.exists()).toBe(true);
      expect(cycleNumber.text()).toContain(state.cycleNumber);
    });
  });
});
